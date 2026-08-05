#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "Eigen/Geometry"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class ManipulationCloudFilterNode : public rclcpp::Node
{
public:
  ManipulationCloudFilterNode()
  : Node("manipulation_cloud_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Filtro dedicado para MoveIt/manipulacion. A diferencia del filtro de
    // navegacion, este no alimenta obstacle_detector ni planning_map; solo
    // publica la nube que usa el octomap de MoveIt.
    declare_parameter<std::string>("input_topic", "/hardware/camera/points");
    declare_parameter<std::string>("output_topic", "/manipulation/camera/points_filtered");
    declare_parameter<std::string>("base_link_name", "base_link");
    declare_parameter<std::string>("input_qos_mode", "dual");
    // En el launch final arranca apagado. El planner lo prende solo cuando
    // necesita calcular una trayectoria de brazo.
    declare_parameter<bool>("active_on_start", false);

    // ROI en base_link. El rango vertical evita piso y puntos por encima del
    // volumen util de los brazos para no contaminar el octomap de MoveIt.
    declare_parameter<double>("roi_x_min_m", 0.15);
    declare_parameter<double>("roi_x_max_m", 1.25);
    declare_parameter<double>("roi_y_min_m", -0.75);
    declare_parameter<double>("roi_y_max_m", 0.75);
    declare_parameter<double>("roi_z_min_m", 0.45);
    declare_parameter<double>("roi_z_max_m", 1.30);

    declare_parameter<double>("voxel_leaf_size", 0.025);
    declare_parameter<int>("input_downsampling", 2);
    declare_parameter<int>("min_points_per_voxel", 2);
    declare_parameter<int>("min_neighbor_voxels", 1);
    declare_parameter<double>("processing_rate_hz", 15.0);
    declare_parameter<double>("tf_fallback_warn_threshold_sec", 0.20);
    declare_parameter<double>("status_period_sec", 2.0);
    declare_parameter<bool>("publish_debug_markers", true);
    declare_parameter<std::string>("debug_marker_topic", "cloud_filter/debug_markers");
    declare_parameter<bool>("debug", false);
    // Hueco temporal para el objeto que se va a tomar. Lo configura
    // manipulation_planner antes de planear el acercamiento, para que MoveIt no
    // trate al propio objetivo como obstaculo.
    declare_parameter<bool>("target_exclusion_enabled", false);
    declare_parameter<double>("target_exclusion_x", 0.0);
    declare_parameter<double>("target_exclusion_y", 0.0);
    declare_parameter<double>("target_exclusion_z", 0.0);
    declare_parameter<double>("target_exclusion_radius_xy", 0.14);
    declare_parameter<double>("target_exclusion_height_z", 0.24);

    load_parameters();

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ManipulationCloudFilterNode::on_parameter_change, this, std::placeholders::_1));

    // MoveIt/RViz suelen suscribirse reliable; publicar la salida compacta asi
    // evita incompatibilidades de QoS con el monitor de octomap.
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(5).reliable());
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_marker_topic_, rclcpp::QoS(1).transient_local());

    // Bajo namespace /manipulation este servicio queda como
    // /manipulation/cloud_filter/enable.
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "cloud_filter/enable",
      std::bind(
        &ManipulationCloudFilterNode::enable_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto processing_period = std::chrono::duration<double>(1.0 / processing_rate_hz_);
    processing_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(processing_period),
      std::bind(&ManipulationCloudFilterNode::process_latest_cloud, this));

    marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ManipulationCloudFilterNode::publish_debug_marker, this));

    const auto status_period = std::chrono::duration<double>(status_period_sec_);
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
      std::bind(&ManipulationCloudFilterNode::publish_status_if_needed, this));

    set_enabled(active_on_start_, false);

    RCLCPP_INFO(
      get_logger(),
      "[manipulation_cloud_filter] input=%s output=%s active=%s roi=[%.2f %.2f] [%.2f %.2f] [%.2f %.2f]",
      input_topic_.c_str(),
      output_topic_.c_str(),
      enabled_ ? "true" : "false",
      roi_x_min_, roi_x_max_,
      roi_y_min_, roi_y_max_,
      roi_z_min_, roi_z_max_);
  }

private:
  struct VoxelKey
  {
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      const auto hx = std::hash<int>{}(key.x * 73856093);
      const auto hy = std::hash<int>{}(key.y * 19349663);
      const auto hz = std::hash<int>{}(key.z * 83492791);
      return hx ^ (hy << 1) ^ (hz << 2);
    }
  };

  struct VoxelAccumulator
  {
    int count = 0;
    double input_x = 0.0;
    double input_y = 0.0;
    double input_z = 0.0;
  };

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_best_effort_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_reliable_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::mutex latest_cloud_mutex_;
  uint64_t latest_cloud_sequence_{0};
  uint64_t processed_cloud_sequence_{0};

  std::atomic<uint64_t> received_clouds_{0};
  std::atomic<uint64_t> processed_clouds_{0};
  std::atomic<uint64_t> published_clouds_{0};
  std::atomic<uint64_t> empty_input_clouds_{0};
  std::atomic<uint64_t> invalid_field_clouds_{0};
  std::atomic<uint64_t> tf_failed_clouds_{0};
  std::atomic<std::size_t> last_total_points_{0};
  std::atomic<std::size_t> last_finite_points_{0};
  std::atomic<std::size_t> last_in_roi_points_{0};
  std::atomic<std::size_t> last_voxels_{0};
  std::atomic<std::size_t> last_published_points_{0};
  uint64_t last_status_received_{0};
  uint64_t last_status_published_{0};

  std::string input_topic_;
  std::string output_topic_;
  std::string base_link_name_;
  std::string input_qos_mode_;
  std::string debug_marker_topic_;

  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double roi_z_min_;
  double roi_z_max_;
  double voxel_leaf_size_;
  double processing_rate_hz_;
  double tf_fallback_warn_threshold_sec_;
  double status_period_sec_;
  int input_downsampling_;
  int min_points_per_voxel_;
  int min_neighbor_voxels_;
  bool active_on_start_;
  bool enabled_{false};
  bool publish_debug_markers_;
  bool debug_;
  bool target_exclusion_enabled_;
  double target_exclusion_x_;
  double target_exclusion_y_;
  double target_exclusion_z_;
  double target_exclusion_radius_xy_;
  double target_exclusion_height_z_;

  void load_parameters()
  {
    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("base_link_name", base_link_name_);
    get_parameter("input_qos_mode", input_qos_mode_);
    get_parameter("active_on_start", active_on_start_);
    get_parameter("roi_x_min_m", roi_x_min_);
    get_parameter("roi_x_max_m", roi_x_max_);
    get_parameter("roi_y_min_m", roi_y_min_);
    get_parameter("roi_y_max_m", roi_y_max_);
    get_parameter("roi_z_min_m", roi_z_min_);
    get_parameter("roi_z_max_m", roi_z_max_);
    get_parameter("voxel_leaf_size", voxel_leaf_size_);
    get_parameter("input_downsampling", input_downsampling_);
    get_parameter("min_points_per_voxel", min_points_per_voxel_);
    get_parameter("min_neighbor_voxels", min_neighbor_voxels_);
    get_parameter("processing_rate_hz", processing_rate_hz_);
    get_parameter("tf_fallback_warn_threshold_sec", tf_fallback_warn_threshold_sec_);
    get_parameter("status_period_sec", status_period_sec_);
    get_parameter("publish_debug_markers", publish_debug_markers_);
    get_parameter("debug_marker_topic", debug_marker_topic_);
    get_parameter("debug", debug_);
    get_parameter("target_exclusion_enabled", target_exclusion_enabled_);
    get_parameter("target_exclusion_x", target_exclusion_x_);
    get_parameter("target_exclusion_y", target_exclusion_y_);
    get_parameter("target_exclusion_z", target_exclusion_z_);
    get_parameter("target_exclusion_radius_xy", target_exclusion_radius_xy_);
    get_parameter("target_exclusion_height_z", target_exclusion_height_z_);
    sanitize_parameters();
  }

  void sanitize_parameters()
  {
    if (roi_x_min_ > roi_x_max_) std::swap(roi_x_min_, roi_x_max_);
    if (roi_y_min_ > roi_y_max_) std::swap(roi_y_min_, roi_y_max_);
    if (roi_z_min_ > roi_z_max_) std::swap(roi_z_min_, roi_z_max_);
    voxel_leaf_size_ = std::max(0.005, voxel_leaf_size_);
    input_downsampling_ = std::max(1, input_downsampling_);
    min_points_per_voxel_ = std::max(1, min_points_per_voxel_);
    min_neighbor_voxels_ = std::max(0, min_neighbor_voxels_);
    processing_rate_hz_ = std::max(1.0, processing_rate_hz_);
    tf_fallback_warn_threshold_sec_ = std::max(0.0, tf_fallback_warn_threshold_sec_);
    status_period_sec_ = std::max(0.5, status_period_sec_);
    target_exclusion_radius_xy_ = std::max(0.0, target_exclusion_radius_xy_);
    target_exclusion_height_z_ = std::max(0.0, target_exclusion_height_z_);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
      const auto & name = param.get_name();
      if (name == "base_link_name") base_link_name_ = param.as_string();
      else if (name == "roi_x_min_m") roi_x_min_ = param.as_double();
      else if (name == "roi_x_max_m") roi_x_max_ = param.as_double();
      else if (name == "roi_y_min_m") roi_y_min_ = param.as_double();
      else if (name == "roi_y_max_m") roi_y_max_ = param.as_double();
      else if (name == "roi_z_min_m") roi_z_min_ = param.as_double();
      else if (name == "roi_z_max_m") roi_z_max_ = param.as_double();
      else if (name == "voxel_leaf_size") voxel_leaf_size_ = param.as_double();
      else if (name == "input_downsampling") input_downsampling_ = static_cast<int>(param.as_int());
      else if (name == "min_points_per_voxel") {
        min_points_per_voxel_ = static_cast<int>(param.as_int());
      } else if (name == "min_neighbor_voxels") {
        min_neighbor_voxels_ = static_cast<int>(param.as_int());
      } else if (name == "processing_rate_hz") {
        processing_rate_hz_ = param.as_double();
      } else if (name == "tf_fallback_warn_threshold_sec") {
        tf_fallback_warn_threshold_sec_ = param.as_double();
      } else if (name == "status_period_sec") {
        status_period_sec_ = param.as_double();
      } else if (name == "publish_debug_markers") {
        publish_debug_markers_ = param.as_bool();
      } else if (name == "debug") {
        debug_ = param.as_bool();
      } else if (name == "target_exclusion_enabled") {
        target_exclusion_enabled_ = param.as_bool();
      } else if (name == "target_exclusion_x") {
        target_exclusion_x_ = param.as_double();
      } else if (name == "target_exclusion_y") {
        target_exclusion_y_ = param.as_double();
      } else if (name == "target_exclusion_z") {
        target_exclusion_z_ = param.as_double();
      } else if (name == "target_exclusion_radius_xy") {
        target_exclusion_radius_xy_ = param.as_double();
      } else if (name == "target_exclusion_height_z") {
        target_exclusion_height_z_ = param.as_double();
      } else if (name == "active_on_start") {
        set_enabled(param.as_bool(), true);
      } else if (
        name == "input_topic" || name == "output_topic" ||
        name == "input_qos_mode" || name == "debug_marker_topic") {
        result.successful = false;
        result.reason = "Topic and QoS parameters require node restart";
        break;
      }
    }

    sanitize_parameters();
    return result;
  }

  void create_input_subscriptions()
  {
    const auto cb =
      std::bind(&ManipulationCloudFilterNode::callback_cloud, this, std::placeholders::_1);
    if (input_qos_mode_ == "reliable") {
      sub_cloud_reliable_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::QoS(5).reliable(), cb);
      sub_cloud_best_effort_.reset();
    } else if (input_qos_mode_ == "best_effort") {
      sub_cloud_best_effort_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(), cb);
      sub_cloud_reliable_.reset();
    } else {
      sub_cloud_best_effort_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(), cb);
      sub_cloud_reliable_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::QoS(5).reliable(), cb);
    }
  }

  void clear_input_subscriptions()
  {
    sub_cloud_best_effort_.reset();
    sub_cloud_reliable_.reset();
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    latest_cloud_.reset();
    ++latest_cloud_sequence_;
  }

  void set_enabled(bool enabled, bool log_change)
  {
    // Apagado significa "sin suscripcion al Kinect". Asi no consumimos CPU ni
    // llenamos el octomap mientras no hay planeacion de manipulacion.
    if (enabled_ == enabled &&
        ((enabled && (sub_cloud_best_effort_ || sub_cloud_reliable_)) ||
         (!enabled && !sub_cloud_best_effort_ && !sub_cloud_reliable_))) {
      return;
    }

    enabled_ = enabled;
    if (enabled_) {
      create_input_subscriptions();
    } else {
      clear_input_subscriptions();
      clear_debug_marker();
    }

    if (log_change) {
      RCLCPP_INFO(
        get_logger(),
        "[manipulation_cloud_filter] %s",
        enabled_ ? "enabled" : "disabled");
    }
  }

  void enable_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    set_enabled(request->data, true);
    response->success = true;
    response->message =
      enabled_ ? "manipulation_cloud_filter_enabled" : "manipulation_cloud_filter_disabled";
  }

  void callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!enabled_ || !msg) {
      return;
    }
    received_clouds_.fetch_add(1);
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    latest_cloud_ = msg;
    ++latest_cloud_sequence_;
  }

  void process_latest_cloud()
  {
    if (!enabled_) {
      return;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
      if (!latest_cloud_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "[manipulation_cloud_filter] enabled but no input clouds on %s",
          input_topic_.c_str());
        return;
      }
      if (latest_cloud_sequence_ == processed_cloud_sequence_) {
        return;
      }
      msg = latest_cloud_;
      sequence = latest_cloud_sequence_;
    }

    process_cloud(*msg);
    processed_cloud_sequence_ = sequence;
  }

  static bool get_float32_field_offset(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & field_name,
    std::size_t & offset)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == field_name &&
          field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        offset = field.offset;
        return true;
      }
    }
    return false;
  }

  bool lookup_cloud_transform(
    const std_msgs::msg::Header & header,
    Eigen::Affine3d & cloud_to_base)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        base_link_name_,
        header.frame_id,
        header.stamp,
        rclcpp::Duration::from_seconds(0.03));
      cloud_to_base = tf2::transformToEigen(tf.transform);
      return true;
    } catch (const std::exception & exact_error) {
      try {
        // Si la nube llega ligeramente adelantada respecto a TF, usar la TF mas
        // reciente mantiene vivo el filtro y evita perder frames completos.
        const auto tf = tf_buffer_.lookupTransform(
          base_link_name_,
          header.frame_id,
          rclcpp::Time(0),
          rclcpp::Duration::from_seconds(0.03));
        cloud_to_base = tf2::transformToEigen(tf.transform);
        const double tf_delta_sec =
          std::fabs((rclcpp::Time(header.stamp) - rclcpp::Time(tf.header.stamp)).seconds());
        if (tf_delta_sec > tf_fallback_warn_threshold_sec_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "[manipulation_cloud_filter] exact TF failed from '%s' to '%s' by %.3f s, using latest TF: %s",
            header.frame_id.c_str(),
            base_link_name_.c_str(),
            tf_delta_sec,
            exact_error.what());
        }
        return true;
      } catch (const std::exception & latest_error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[manipulation_cloud_filter] TF lookup failed from '%s' to '%s': %s",
          header.frame_id.c_str(),
          base_link_name_.c_str(),
          latest_error.what());
        return false;
      }
    }
  }

  bool point_in_roi(const Eigen::Vector3d & point) const
  {
    return point.x() >= roi_x_min_ && point.x() <= roi_x_max_ &&
           point.y() >= roi_y_min_ && point.y() <= roi_y_max_ &&
           point.z() >= roi_z_min_ && point.z() <= roi_z_max_;
  }

  bool point_in_target_exclusion(const Eigen::Vector3d & point) const
  {
    // Cilindro vertical en base_link: radio en XY y altura en Z. Es mas estable
    // que depender del bbox 2D cuando ya tenemos la pose 3D del objeto.
    if (!target_exclusion_enabled_) {
      return false;
    }

    const double dx = point.x() - target_exclusion_x_;
    const double dy = point.y() - target_exclusion_y_;
    const double dz = std::fabs(point.z() - target_exclusion_z_);
    const double radius_sq = target_exclusion_radius_xy_ * target_exclusion_radius_xy_;
    return (dx * dx + dy * dy) <= radius_sq &&
           dz <= target_exclusion_height_z_ * 0.5;
  }

  int count_neighbors(
    const std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> & voxels,
    const VoxelKey & key) const
  {
    int neighbors = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          VoxelKey neighbor{key.x + dx, key.y + dy, key.z + dz};
          if (voxels.find(neighbor) != voxels.end()) {
            ++neighbors;
          }
        }
      }
    }
    return neighbors;
  }

  void process_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    processed_clouds_.fetch_add(1);

    const auto total_points =
      static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
    last_total_points_ = total_points;
    if (total_points == 0) {
      empty_input_clouds_.fetch_add(1);
      publish_filtered_cloud(cloud, {});
      return;
    }

    std::size_t x_offset;
    std::size_t y_offset;
    std::size_t z_offset;
    if (!get_float32_field_offset(cloud, "x", x_offset) ||
        !get_float32_field_offset(cloud, "y", y_offset) ||
        !get_float32_field_offset(cloud, "z", z_offset)) {
      invalid_field_clouds_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "[manipulation_cloud_filter] input cloud does not expose float32 x/y/z fields");
      return;
    }

    Eigen::Affine3d cloud_to_base;
    if (!lookup_cloud_transform(cloud.header, cloud_to_base)) {
      tf_failed_clouds_.fetch_add(1);
      return;
    }

    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
    voxels.reserve(total_points / static_cast<std::size_t>(input_downsampling_) + 1);

    // Cada punto se transforma a base_link solo para decidir si cae dentro del
    // ROI y para calcular el voxel. La nube publicada conserva el frame original
    // del sensor, igual que la nube cruda, para que MoveIt use su TF normal.
    const auto * data = cloud.data.data();
    std::size_t finite_points = 0;
    std::size_t in_roi_points = 0;
    std::size_t target_excluded_points = 0;

    for (std::size_t i = 0; i < total_points; i += static_cast<std::size_t>(input_downsampling_)) {
      const auto * point_ptr = data + i * cloud.point_step;
      float x;
      float y;
      float z;
      std::memcpy(&x, point_ptr + x_offset, sizeof(float));
      std::memcpy(&y, point_ptr + y_offset, sizeof(float));
      std::memcpy(&z, point_ptr + z_offset, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      ++finite_points;

      const Eigen::Vector3d point_base =
        cloud_to_base * Eigen::Vector3d(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
      if (!point_in_roi(point_base)) {
        continue;
      }
      if (point_in_target_exclusion(point_base)) {
        ++target_excluded_points;
        continue;
      }
      ++in_roi_points;

      VoxelKey key{
        static_cast<int>(std::floor(point_base.x() / voxel_leaf_size_)),
        static_cast<int>(std::floor(point_base.y() / voxel_leaf_size_)),
        static_cast<int>(std::floor(point_base.z() / voxel_leaf_size_))};
      auto & acc = voxels[key];
      ++acc.count;
      acc.input_x += x;
      acc.input_y += y;
      acc.input_z += z;
    }

    std::vector<VoxelAccumulator> accepted_voxels;
    accepted_voxels.reserve(voxels.size());
    for (const auto & item : voxels) {
      const auto & key = item.first;
      auto acc = item.second;
      if (acc.count < min_points_per_voxel_) {
        continue;
      }
      if (min_neighbor_voxels_ > 0 && count_neighbors(voxels, key) < min_neighbor_voxels_) {
        continue;
      }
      // Publicamos un punto promedio por voxel aceptado. Esto reduce densidad
      // sin cambiar el volumen de obstaculos que vera MoveIt.
      const double inv_count = 1.0 / static_cast<double>(acc.count);
      acc.input_x *= inv_count;
      acc.input_y *= inv_count;
      acc.input_z *= inv_count;
      accepted_voxels.push_back(acc);
    }

    last_finite_points_ = finite_points;
    last_in_roi_points_ = in_roi_points;
    last_voxels_ = voxels.size();
    last_published_points_ = accepted_voxels.size();

    if (debug_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[manipulation_cloud_filter] input=%zu finite=%zu in_roi=%zu voxels=%zu accepted=%zu",
        total_points,
        finite_points,
        in_roi_points,
        voxels.size(),
        accepted_voxels.size());
    }

    if (target_exclusion_enabled_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[manipulation_cloud_filter] target_exclusion center=[%.3f %.3f %.3f] radius_xy=%.3f height_z=%.3f excluded=%zu accepted=%zu",
        target_exclusion_x_,
        target_exclusion_y_,
        target_exclusion_z_,
        target_exclusion_radius_xy_,
        target_exclusion_height_z_,
        target_excluded_points,
        accepted_voxels.size());
    }

    publish_filtered_cloud(cloud, accepted_voxels);
  }

  void publish_filtered_cloud(
    const sensor_msgs::msg::PointCloud2 & input,
    const std::vector<VoxelAccumulator> & accepted_voxels)
  {
    // Salida XYZ compacta: suficiente para el PointCloudOctomapUpdater y mas
    // ligera que reenviar la nube organizada completa.
    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.height = 1;
    output.width = static_cast<uint32_t>(accepted_voxels.size());
    output.is_bigendian = false;
    output.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(accepted_voxels.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    for (const auto & point : accepted_voxels) {
      *out_x = static_cast<float>(point.input_x);
      *out_y = static_cast<float>(point.input_y);
      *out_z = static_cast<float>(point.input_z);
      ++out_x;
      ++out_y;
      ++out_z;
    }

    pub_cloud_->publish(output);
    published_clouds_.fetch_add(1);
  }

  visualization_msgs::msg::Marker make_roi_marker() const
  {
    // Marcador unico del prisma filtrado. Es solo debug visual; el octomap se
    // alimenta exclusivamente de output_topic_.
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_link_name_;
    marker.header.stamp.sec = 0;
    marker.header.stamp.nanosec = 0;
    marker.ns = "manipulation_cloud_filter";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = true;
    marker.pose.position.x = (roi_x_min_ + roi_x_max_) * 0.5;
    marker.pose.position.y = (roi_y_min_ + roi_y_max_) * 0.5;
    marker.pose.position.z = (roi_z_min_ + roi_z_max_) * 0.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.001, roi_x_max_ - roi_x_min_);
    marker.scale.y = std::max(0.001, roi_y_max_ - roi_y_min_);
    marker.scale.z = std::max(0.001, roi_z_max_ - roi_z_min_);
    marker.color.r = 0.0f;
    marker.color.g = 0.85f;
    marker.color.b = 1.0f;
    marker.color.a = 0.18f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  void publish_debug_marker()
  {
    if (!enabled_ || !publish_debug_markers_) {
      return;
    }
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_roi_marker());
    pub_markers_->publish(markers);
  }

  void clear_debug_marker()
  {
    visualization_msgs::msg::MarkerArray markers;
    auto marker = make_roi_marker();
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(marker);
    pub_markers_->publish(markers);
  }

  void publish_status_if_needed()
  {
    if (!enabled_) {
      return;
    }

    const auto received = received_clouds_.load();
    const auto published = published_clouds_.load();
    if (received == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[manipulation_cloud_filter] no input clouds on '%s' (publishers=%zu, qos=%s)",
        input_topic_.c_str(),
        count_publishers(input_topic_),
        input_qos_mode_.c_str());
      return;
    }

    const bool received_new = received != last_status_received_;
    const bool published_new = published != last_status_published_;
    if (received_new && !published_new) {
      RCLCPP_WARN(
        get_logger(),
        "[manipulation_cloud_filter] receiving but not publishing. rx=%lu processed=%lu pub=%lu empty=%lu bad_fields=%lu tf_fail=%lu last(raw=%zu finite=%zu in_roi=%zu voxels=%zu accepted=%zu)",
        received,
        processed_clouds_.load(),
        published,
        empty_input_clouds_.load(),
        invalid_field_clouds_.load(),
        tf_failed_clouds_.load(),
        last_total_points_.load(),
        last_finite_points_.load(),
        last_in_roi_points_.load(),
        last_voxels_.load(),
        last_published_points_.load());
    }

    last_status_received_ = received;
    last_status_published_ = published;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulationCloudFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

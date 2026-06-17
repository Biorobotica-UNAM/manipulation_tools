# manipulation_tools

Launch y utilidades comunes para levantar la pila de manipulacion de Justina.

Este paquete conecta las piezas principales:

* MoveIt (`move_group`)
* `manipulation_planner`
* `manipulation_controller`
* filtro de nube dedicado a manipulacion

## Que hace ahora

El launch principal es:

* `launch/manipulation.launch.xml`

Al lanzarse bajo el namespace `manipulation`, deja la cadena lista para que
acciones como `Take.cpp` puedan pedir una trayectoria, ejecutarla y observar el
estado real del robot.

## Remapeo de joint states

La fuente canonica del estado articular es:

* `/hardware/joint_states`

El launch remapea:

* `joint_states` -> `/hardware/joint_states`
* `/manipulation/joint_states` -> `/hardware/joint_states`

Esto evita que MoveIt o nodos antiguos consuman una fuente de joints distinta a
la del hardware real.

## Filtro de nube

El argumento:

* `use_cloud_filter`

controla si se lanza el filtro dedicado a manipulacion. En robot real puede
mantenerse activo para poblar/limpiar el octomap de MoveIt con una nube ya
recortada. En simulacion puede usarse `use_cloud_filter:=false` para probar
solo la cadena de planeacion y ejecucion.

## Cadena resultante

La ruta esperada para tomar objetos es:

1. `executor_tools/Take.cpp` decide objetivo, brazo, tip y candidatos.
2. `manipulation_planner` pide una trayectoria a MoveIt.
3. MoveIt planea con OMPL y RRTConnect.
4. `manipulation_controller` ejecuta los bloques de torso + brazo.
5. El hardware real, o el simulador, publica el nuevo estado en
   `/hardware/joint_states`.

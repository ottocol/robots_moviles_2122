# Introducción a ROS como usuario

ROS (Robot Operating System) es un conjunto de bibliotecas y herramientas para ayudar a los desarrolladores de software crear aplicaciones robóticas. Proporciona una abstracción del hardware, de los controladores de dispositivos, las bibliotecas, visualizadores, paso de mensajes, gestión de paquetes y mucho más.

Ya habéis utilizado ROS en otras asignaturas así que nos limitaremos a dar un breve repaso de los conceptos en que se basa, sin explicarlos en profundidad.

A continuación veremos cómo ejecutar programas ROS en los simuladores y cómo funcionan los nodos y topics de ROS. De momento no veremos cómo programar en ROS ya que lo dejamos para sesiones posteriores.

## Conceptos básicos

- Tópico (*topic*): Son canales de información entre los nodos. Un nodo puede emitir o suscribirse a un tópico. Por ejemplo, stage (simulador de robots) emite un tópico que es la odometría del robot. Cualquier nodos se puede suscribir. El nodo que emite no controla quién está suscrito. La información es, por tanto, unidireccional (asíncrona). Si lo que queremos es una comunicación síncrona (petición/respuesta) debemos usar servicios. Un tópico no es más que un mensaje que se envía. Podemos usar distintos tipos de clases de mensajes.
- Paquete (*package*): El software en ROS está organizado en paquetes. Un paquete puede contener un nodo, una librería, conjunto de datos, o cualquier cosa que pueda constituir un módulo. Los paquetes pueden organizarse en pilas (stacks).
- Nodo (*node*): Un nodo es un proceso que realiza algún tipo de compu- tación en el sistema. Los nodos se combinan dentro de un grafo, compar- tiendo información entre ellos, para crear ejecuciones complejas. Un nodo puede controlar un sensor láser, otro los motores de un robot y otro la construcción de mapas.
- Pila (*stack*): Conjunto de nodos que juntos proporcionan alguna funcionalidad. Por ejemplo, la *pila de navegación* sirve para que el robot pueda moverse a un punto del mapa por la ruta más corta y evitando obstáculos por el camino.

## Configuración inicial para el laboratorio

> ANTES de empezar con la práctica, **solo si estás usando los ordenadores del laboratorio**, abre una terminal y copia y pega estas instrucciones:

```bash
echo "source $HOME/tb2_ws/devel_isolated/setup.bash" >> ~/.bashrc
echo "source $HOME/ws_rviz_new/devel/setup.bash" >> ~/.bashrc
```
> Y a continuación **cierra esta terminal y abre una nueva** para que los cambios tengan efecto. *EXPLICACIÓN: la primera línea añade soporte para el robot **turtlebot 2**, la segunda hace que se use una versión modificada de rviz para solucionar una incompatibilidad con las tarjetas gráficas del laboratorio. Si has instalado ROS en tu propio equipo con las instrucciones que damos en la asignatura, o bien usas la máquina virtual que proporcionamos, ya tendrás estas instrucciones en tu `.bashrc` y por tanto no son necesarias.*


## Probando un robot en el simulador Stage

Probar una aplicación por primera vez en un robot real suele ser problemático, ya que depurar el código es complicado y además el  robot podría sufrir daños si el código no funciona correctamente . Por eso la práctica habitual es probar el código primero en un simulador y luego trasladarlo al robot real cuando estamos razonablemente seguros de que va a funcionar.

ROS está integrado con diversos simuladores. Nosotros usaremos dos distintos:

- **Stage**: es un simulador 2D, útil porque consume pocos recursos y es suficiente para ciertos casos. Por ejemplo un robot cuyo sensor principal sea un láser o un anillo de sonares básicamente obtiene información 2D del mundo.
- **Gazebo**: es un simulador multirobot de entornos 3D, mucho más realista que stage, aunque también consume muchos más recursos computacionales.

Vamos a usar primero **Stage** por ser más simple. Para ejecutar el simulador necesitamos un fichero de definición de mundo, en el que se especifica cómo es el entorno (dimensiones, paredes, obstáculos) y el robot (dimensiones físicas y sensores). En la web de la asignatura podéis descargar un ejemplo de mundo en un zip que hay que descomprimir. 

El fichero de definición del mundo propiamente dicho es el `.world`. Además hay un fichero con un *bitmap* (en este ejemplo un `.pgm`) en el que se define el espacio vacío/ocupado.

Para ejecutar *stage* con el ejemplo, teclear en una terminal (desde el mismo directorio donde hayáis descomprimido el `.zip`):

```bash
# Necesario para "arrancar" ROS. Ojo, el & hace que el proceso se quede en background
# y podamos seguir tecleando sin abrir otra terminal
roscore & 
# Ejecutar el simulador
rosrun stage_ros stageros ejemplo.world
```

Debería aparecer una ventana 2D con el mundo simulado. Si abrís el `ejemplo.pgm` notaréis que es realmente el mapa del mundo (`pgm` es un formato gráfico para imágenes en escala de grises, aunque no es probable que os suene porque no se usa mucho actualmente). En el `.world`, que usa una [sintaxis propia de Stage](https://player-stage-manual.readthedocs.io/en/latest/WORLDFILES/), se especifican sus dimensiones y las del robot  en metros junto con los parámetros físicos del robot y de sus sensores.

> Fijaos en que crear un nuevo mundo simulado es tan sencillo como crear una imagen en blanco y negro en cualquier aplicación (negro para los obstáculos/paredes, blanco para espacio vacío), guardarla en formato `.pgm` (casi todos los programas de dibujo/apps gráficas son compatibles) y en el fichero `.world` ajustar el tamaño en metros si lo deseáis. Para cambiar los sensores/propiedades del robot tendríais que miraros [la sintaxis de `.world`](https://player-stage-manual.readthedocs.io/en/latest/WORLDFILES/) en la documentación de Stage). En el caso del `ejemplo.world` el mundo simulado se carga de las líneas 51 a la 57

```bash
# load an environment bitmap
floorplan
( 
  name "rm"
  bitmap "rm.pgm"
  size [40.0 20.0 1.0] #tamaño del mundo en metros en x,y,z. z sería el "alto de las paredes"
)
```

> En `/opt/ros/noetic/share/stage/worlds` y `/opt/ros/noetic/share/stage_ros/world` tenéis muchos otros ficheros de mundos de ejemplo, algunos multirobot o con sensores adicionales como cámaras (aunque simplificadas porque la simulación de Stage no es 3D sino 2.5D). Tened en cuenta que tenéis que poner siempre **la trayectoria  hasta el fichero .world** ya que no se busca en estos directorios por defecto, por ejemplo `rosrun stage_ros stageros /opt/ros/noetic/share/stage_ros/world/willow-erratic.world`.

## Nodos y topics

### Viendo topics en modo texto

Por supuesto un robot no ve directamente el entorno, sea real o simulado, sino que lo percibe indirectamente a través de sus *sensores*. Como todo en ROS, la información de los sensores es accesible a través de ciertos *topics* en los que Stage publica la información. Abre una terminal aparte (para que el simulador siga ejecutándose en la anterior) y escribe:

```bash
rostopic list
```
Veremos una lista de *topics* publicados. Por ejemplo `/odom` es la odometría del robot, un sensor que nos dice en qué coordenadas se encuentra con respecto al punto inicial del que partió. Como todavía el robot no se ha movido, si imprimes los mensajes de este *topic* deben indicar que está en la posición `0,0,0)`. Pruébalo con:

```bash
#con el -n 1 imprimimos solo un mensaje 
#para que no se nos llene la pantalla de datos
rostopic echo /odom -n 1
```
### Viendo el  grafo de nodos con `rqt_graph`

Para ver la información de modo gráfico puedes usar la orden `rqt_graph`. Pruébala, verás que solo aparece un nodo, para que aparezca el resto como en la siguiente figura tendrás que:

- En el desplegable donde pone `Nodes only` cambiarlo por `Nodes/topics (all)` y darle al botón de recargar (con la flecha circular) 
- En la opción de `Hide` desmarcar las casillas `Dead sinks` y `Leaf Topics`, así puedes ver los nodos que reciben pero no publican mensajes y los que publican pero no reciben, respectivamente

![](rqt_graph.png)

### Publicando *topics* manualmente

En el grafo de nodos y *topics* habrás visto un *topic* llamado `/cmd_vel`. En este *topic* están escuchando los "motores" del robot, y si publicamos en él estaremos por tanto moviéndolo. En la terminal ejecuta:

```bash
rostopic info /cmd_vel
```
Para ver de qué tipo es el *topic*. Verás que nos dice quién lo publica, quién  está suscrito y de qué tipo es el mensaje. En ROS hay una serie de tipos de mensajes predefinidos y también  el programador se puede definir los suyos propios. En nuestro caso el tipo es `geometry_msgs/Twist`. Ahora para ver información sobre qué datos componen un mensaje de ese tipo, escribe en la terminal:

```bash
rosmsg show geometry_msgs/Twist
```

verás que te dice que un mensaje de este tipo está compuesto de 2 vectores 3D con componentes llamados `x`, `y`, `z`. El primer vector se llama `linear` y representa la velocidad lineal y el segundo `angular` y como es lógico representa la velocidad angular.

Normalmente se haría por código pero también podemos mover al robot publicando mensajes de manera manual. Prueba:

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

El `-r 10` lo que hará es publicar el mensaje 10 veces por segundo. Verás que el robot empieza a moverse hacia el frente (eje X) a una velocidad de 0.2 m/s. El comando se quedará ejecutando (y el robot moviéndose, si no choca) hasta que pulses `Ctrl-C` para pararlo.

### Viendo topics en modo gráfico

Entre los topics que publica el simulador, `/base_scan` es el sensor de rango del robot. En el `.world` de ejemplo se define un sensor de tipo *laser scan* que da las distancias a los objetos más cercanos en un sector de 270 grados. Podemos ver la información en modo numérico en la terminal con 

```bash
rostopic echo /base_scan
```

No obstante ver impresa la lista de números con las distancias no es muy intuitivo. En general es mucho mejor visualizar la información de los sensores en modo gráfico. Para ello disponemos en ROS de la herramienta `rviz`.

```bash
rosrun rviz rviz
```

> **OJO** si no ha salido la ventana de rviz y en su lugar ha aparecido un error en rojo "Unable to create the rendering window..." junto con muchos más mensajes anteriores, es posible que no hayas hecho lo que pone en el apartado "Configuración inicial para el laboratorio".

Al entrar en `rviz` lo primero es **cambiar en el panel izquierdo la opción `fixed frame` en las `Global Options`**. Este es el sistema de coordenadas que usará `rviz` para dibujar. Ahora está puesto a `map` y da un error porque eso sería para un mapa construido por el robot, cosa que no se ha hecho (lo haremos en una práctica posterior). Lo podéis cambiar por cualquiera de las otras opciones que sale al seleccionar el desplegable a la derecha de `fixed frame`, por ejemplo `odom`.

Podemos visualizar el sensor de rango añadiendo un nuevo *display* de tipo *Laserscan* (botón `Add` de la parte inferior del panel izquierdo). Una vez añadido hay que cambiar la opción `topic` para que se corresponda con el que está publicando el robot, en este caso `base_scan`. Debería aparecer dibujado en rojo el entorno que rodea al robot.

> Dado que ambas muestran información gráfica es fácil confundirse en el papel que desempeñan Stage y rviz. Como hemos dicho, Stage es un simulador y rviz un visor de los sensores. Si usáramos el robot real (como haremos la semana que viene) no tendríamos Stage en marcha, pero sí rviz, ya que no habría robot simulado pero sí sensores que visualizar. Además en rviz no veremos el mundo completo sino solo lo que percibe el robot. La confusión tiende a aumentar porque algunos simuladores también permiten visualizar los sensores del robot (en Stage en la opción `View > Data`) y en `rviz` también se puede cargar un mapa del entorno y un modelo 3D del robot si lo tenemos. 

## El simulador Gazebo

[Gazebo](http://gazebosim.org) es un simulador 3D mucho más avanzado que Stage, y es el que necesitaremos para poder simular sensores como cámaras o cámaras RGBD (s las que detectan la profundidad, tipo kinect) o simular efectos físicos como choques, empujar objetos, etc.

Para simular un robot en Gazebo necesitamos una configuración mucho más compleja que con Stage, ya que debemos tener un modelo 3D detallado del robot con todos los parámetros físicos necesarios para la simulación. Por eso usaremos  paquetes ROS distintos para simular cada robot de forma independiente.

### Turtlebot 3 simulado

Los Turtlebot 3 son uno de los modelos de robot más populares en ROS, fueron diseñados con la intención de ser sistemas robóticos de bajo coste manteniendo la compatibilidad con ROS. En el laboratorio no tenemos estos robots pero podemos simularlos en Gazebo para probar distintos algoritmos.

El paquete que implementa la simulación de Turtlebot 3 en Gazebo es, cómo no, `turtlebot3_gazebo`, que incluye varios mundos de ejemplo, vamos a probar uno de ellos:

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

> Hay dos modelos de Turtlebot 3, "burger" y "waffle", el primero es más sencillo, el ordenador de a bordo es una raspberry pi 3 y no lleva cámara por defecto, el *waffle* lleva una cámara RGBD y el ordenador de a bordo es más potente. Ambos llevan un laser que nos da una "rebanada horizontal" del entorno en 360 grados. En cada terminal que usemos la simulación tenemos que decir qué modelo estamos simulando con el `export TURTLEBOT3_MODEL` (tedioso pero necesario). Por cierto, ya que nos cuesta lo mismo 😆, aquí simulamos el *waffle*, el más potente.

Si hacemos `rostopic list` en otra terminal podremos ver que hay una serie de *topics* que comienzan por `/camera`, con los datos de la cámara del modelo *waffle*. Los `/camera/rgb` tienen la información de color mientras que los `/camera/depth` contienen la información 3D. Además tenemos el *topic* `/scan` con los datos del laser.

Los datos de la cámara podemos verlos con un programita que se llama `rqt_image_view` (teclead esta orden en una terminal). Aparecerá una ventana en la que podemos seleccionar el *topic* de la cámara que nos interese. Notad que las imágenes de profundidad se representan en escala de grises, donde los objetos cercanos aparecen en color claro y los lejanos en color oscuro. 

Para ver no solo la cámara sino también el resto de sensores (en el caso del Turtlebot 3, el laser) podemos lanzar `rviz`. En este caso tendremos que hacerlo de una forma un poco especial (no con `rosrun rviz rviz` como antes).

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

> El simulador de Turtlebot 3 no pone en marcha por defecto una serie de nodos que `rviz` necesita para poder calcular ciertas transformaciones entre diversos sistemas de coordenadas. Con la orden anterior además de poner `rviz` en marcha inicializamos los nodos que se encargan de esas transformaciones.

Podemos mover al robot con el teclado con las siguientes órdenes (en otra terminal)

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

> Ojo, para que lo anterior funcione, la terminal donde se ejecute debe tener el foco del teclado (debe ser la ventana activa), o sea que debéis clicar con el ratón antes en ella (puede ser complicado hacer esto y que no se os pierda la ventana del simulador, tendréis que distribuirlas en pantalla)

Aparecerá en pantalla un mensaje indicando cuáles son las teclas de control y qué hace cada una.


### Turtlebot 2 simulado

Los robots que tenemos en el laboratorio son de la generación anterior de Turtlebot, que es la 2.

> Como curiosidad, no es que la versión 3 de Turtlebot sea más potente que la 2, como uno esperaría. De hecho es al contrario, los Turtlebot 2 del laboratorio tienen un hardware bastante más potente que los Turtlebot 3 que hemos simulado antes. Sin embargo, por desgracia los Turtlebot 3 tienen soporte oficial en las nuevas versiones de ROS pero no así los 2.

>El simulador de Turtlebot 2 no está portado oficialmente a ROS Noetic, lo que nos ha obligado a compilarlo desde los fuentes y a tener que usarlo en el laboratorio de una forma un poco particular, ya que no está instalado donde el resto de paquetes de ROS. Básicamente, en cada terminal que hagamos uso de algo relativo a Turtlebot 2 lo primero que tenemos que hacer es un `source tb2_ws/devel_isolated/setup.bash`

El *package* `turtlebot_gazebo` contiene la simulación de los Turtlebot 2 para gazebo, podemos lanzar una simulación con un mundo predefinido escribiendo en una terminal:

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```

Podemos mover al robot mediante el teclado con `turtlebot_teleop` (nótese que la orden es ligeramente distinta a la que usábamos con Turtlebot 3).

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

Si lanzáis la herramienta `rviz` en otra terminal como lo hacíamos con stage podréis ver gráficamente la información de los sensores del robot simulado.

```bash
rosrun rviz rviz
```
# Práctica guiada 2. Introducción a ROS para desarrolladores

Ya vimos los comandos más habituales en ROS y cómo lanzar nodos ya hechos. Aquí vamos a ver cómo crear nuestros propios nodos y organizarlos en paquetes. Usaremos Python en los ejemplos porque el código resulta más claro y se simplifica el proceso de desarrollo, pero también se puede programar usando C++.

> Por un *bug* en la instalación de los ordenadores del laboratorio, de momento no funciona lo de crear workspaces y packages (si tienes tu propia instalación de ROS sí debería hacerlo). No obstante eso no impide probar los programas en python, en realidad no necesitan estar dentro de ningún workspace. Si estás en un ordenador del laboratorio salta directamente al apartado 2.

## 1. *Workspaces* y *packages*

Nuestros programas ROS estarán contenidos en un paquete (*package*) que no es más que un conjunto de programas relacionados entre sí. En ROS hay multitud de paquetes ya implementados . Por ejemplo ya habeís visto `turtlebot_gazebo`, que da soporte para simular el robot turtlebot en Gazebo o `turtlebot_teleop` que permite mover el turtlebot con teclado o mando xbox.

En un nivel superior, los *packages* se agrupan en *workspaces* o espacios de trabajo. Podéis usar durante todo el curso el mismo *workspace*, no es necesario que uséis uno distinto en cada práctica.

### Crear un *workspace*

Primero hay que crear el. *workspace* que no es más que un directorio con una determinada estructura de subdirectorios y unos cuantos archivos de configuración. 

1. **Crea un directorio cualquiera** para alojar el *workspace*. En los ejemplos de ROS se suele usar el nombre `catkin_ws`. **Nosotros usaremos otro distinto, porque los robots del laboratorio ya tienen un  `catkin_ws` creado y modificarlo con el nuestro podría hacer que el robot dejara de funcionar**)

2. Métete el el directorio, dentro de él **crea un subdirectorio `src`** 

    ```bash
      cd mi_ws
      mkdir src
    ```

3. **Ejecuta la orden `catkin_make`** (desde `mi_ws`, no desde el ` src`) para crear automáticamente el resto de directorios y ficheros de configuración del *workspace*

> IMPORTANTE: para que el sistema "sepa" que `mi_ws` es un *workspace* de ROS, cada vez que vayamos a usarlo hay que hacer antes

  ```bash
   source devel/setup.bash
  ```

> Esto tendrá efecto en la terminal actual, si abrimos otra habrá que hacerlo de nuevo

### Crear un *package*

Los paquetes residen el el directorio `src` del *workspace*

1. **Métete en el directorio `src`** del *workspace* que creaste 

      ```bash
      cd mi_ws/src
      ```
2. **Ejecuta la orden `catkin_create_pkg`** para crear el paquete. Hace falta pasarle el nombre del nuevo paquete y la lista de los paquetes de los que depende. En nuestro caso son dos: `rospy`, que nos permite programar en ROS usando Python y `std_msgs`, donde se definen los tipos de mensajes estándar       
  
      ```bash
      catkin_create_pkg practica1 rospy std_msgs
      ```

## 2. Ejemplo productor/consumidor

Como ROS está basado en el paso de mensajes, algunos nodos publicarán mensajes y otros los consumirán, (aunque puede haber alguno que haga las dos cosas). Vamos a ver el típico ejemplo en el que un nodo produce mensajes y otro nodo los consume.

Coloca el siguiente código en un archivo `productor.py`en `practica1/src`, correspondiendo con el productor de mensajes:

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node('productor')
# inicializamos un productor de mensajes. Parámetros:
# - nombre del topic
# - tipo de datos del topic
# - tamaño de la cola de mensajes
pub = rospy.Publisher('saludo', String, queue_size=10)
# definimos la frecuencia de publicación en Hz (mensajes por segundo)
rate = rospy.Rate(1)
count = 0
while not rospy.is_shutdown():
    # publicamos el mensaje
    pub.publish('saludo numero '  + str(count))
    count += 1
    # esperamos según nos diga la frecuencia (1/rate segundos)
    # en este caso 1 segundo
    rate.sleep()
```

Puedes ejecutar el programa llamando al intérprete de python: `python productor.py`.

**IMPORTANTE:** **antes** de ejecutar el programa tienes que asegurarte de que el nodo *master* de ROS está activo. Esto lo puedes hacer ejecutando `roscore`en otra terminal.

> También puedes ejecutarlo directamente con `./productor.py`, que es un poco más cómodo, pero para eso antes tienes que asignarle permisos de ejecución: chmod ugo+x productor.py` (y asegurarte que la primera línea del archivo es `#!/usr/bin/env python`)

Puedes usar la herramienta `rostopic` de ROS para ver los *topics* existentes, información sobre ellos, y los mensajes publicados dentro de cada *topic*. Por ejemplo:

- `rostopic list`mostrará la lista de *topics*. Entre ellos, si está en marcha `productor.py` deberías ver el *topic* llamado `/saludo`
- `rostopic info nombre_del_topic` mostrará información del topic en cuestión, como por ejemplo el tipo de mensajes, y los nodos que publican y consumen este *topic*.
- `rostopic echo nombre_del_topic` mostrará los mensajes que se van publicando. Para terminar, pulsa `Ctrl-C`.

También podemos consumir los mensajes del *topic* con Python, como lo hace el siguiente código, que puedes guardar en `consumidor.py`:

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

# Esta función es la que se ejecutará cada vez que se reciba un mensaje
def callback(msg): 
    print(msg.data)

rospy.init_node('consumidor')
# definimos un consumidor de mensajes. Parámetros:
# - nombre del topic
# - tipo del topic
# - qué función hace de callback, llamándose cada vez que se reciba un mensaje
sub = rospy.Subscriber('saludo', String, callback)
# nos ponemos a escuchar y cedemos el control a ROS
rospy.spin()

```

Recuerda que para ejecutarlo tienes dos posibilidades:

```bash
#opción 1
python  consumidor.py
#opción 2
chmod ugo+x consumidor.py #una sola vez, para dar permisos de ejecución
./consumidor.py
```

Si también está en marcha el `productor.py` deberían aparecer en pantalla los mensajes a medida que se van recibiendo.

## 3. Leyendo y publicando mensajes del robot

Los ejemplos del apartado anterior son ilustrativos del funcionamiento básico de los mensajes en ROS, pero tienen poco que ver con robots. Vamos a ver ejemplos en los que leamos información de los sensores del robot y mandemos información a los efectores.

Lo primero que necesitamos es un robot real o simulado. Recordad que para ejecutar Gazebo simulando un turtlebot podemos teclear:

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```
### Leyendo los mensajes del láser

Vamos a leer y mostrar información sobre el sensor de rango 2D, que detecta distancias a los obstáculos más cercanos alrededor del robot (normalmente en un ángulo centrado al frente). En ROS es típico que la información de este sensor se publique en un topic llamado `/scan` (aunque el nombre puede variar).

> NOTA: el turtlebot simulado, en realidad no tiene un sensor de rango 2D. Tiene uno 3D que es una cámara RGBD (la Kinect). No obstante hay un nodo que se encarga de transformar estos datos a información 2D y publicarlos en el topic `/scan`, que es el que usamos en este ejemplo.

Para escribir el código Python necesitamos saber qué información hay en el topic `/scan`. Esto lo podemos hacer con varios comandos de ROS. Por ejemplo con `rostopic` podemos ver el tipo de datos del topic. **Teniendo la simulación de Gazebo en marcha (o el robot real)** escribe en una terminal:

```bash
rostopic info /scan
```

Aparecerá en pantalla junto con otra información el tipo de datos del *topic*, en este caso `sensor_msgs/LaserScan`. Con el comando `rosmsg`podemos ver la estructura de un mensaje de este tipo

```bash
rosmsg show sensor_msgs/LaserScan
```

En la terminal deberían aparecer los campos que componen el mensaje y sus tipos. Por ejemplo están el alcance mínimo y máximo de las lecturas (`range_min`y `range_max`), el ángulo inicial y final en radianes (`angle_min`y `angle_max`,  por ejemplo -1.5 y 1.5 indicarían aproximadamente que el sensor tiene un campo de visión de 180 grados al frente) y las lecturas del sensor están en un array llamado `ranges`.

> Si queréis  imprimir un mensaje de tipo `/scan`para ver el aspecto que tienen los datos reales podéis hacer `rostopic echo /scan -n1` (el `-n1` hace que se imprima solo 1 mensaje). Seguramente tendrás que hacer *scroll* hacia arriba para ver los datos.

Con esta información ya podemos escribir un pequeño programa ROS que muestre los datos del sensor de rango:

```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

# mostramos la distancia al objeto más cercano y más lejano detectados
def callback(msg): 
    masCercano = 10000
    masLejano = 0
    for lectura in msg.ranges:
        if lectura<masCercano:
            masCercano = lectura
        elif lectura>masLejano:
            masLejano = lectura    
    print('más cercano:', masCercano, 'más lejano:', masLejano)

rospy.init_node('read_scan')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
```

Como nuestro código depende del paquete `sensor_msgs` deberíamos **añadir en el `package.xml` una etiqueta `<depend>`**indicándolo. Hacia el final del archivo habrá una serie de etiquetas de este tipo, podemos añadir una nueva:

```xml
<depend>sensor_msgs</depend>
```

### 4. Publicando mensajes para controlar los motores

Podemos controlar el movimiento del robot publicando mensajes en el *topic* `/mobile_base/commands/velocity` (Turtlebot 2) o `cmd/vel` (Turtlebot3 o simulador Stage) . Mediante el comando `rostopic info <topic_del_motor>`podremos ver que usa mensajes de tipo `geometry_msgs/Twist` y con `rosmsg show geometry_msgs/Twist` podemos ver la estructura del mensaje. 

Básicamente podemos fijar una velocidad lineal en x,y,z y también una velocidad angular con los mismos componentes. Al publicar un mensaje de este tipo en Turtlebot2 no dejamos fija la velocidad sino que pasado un breve espacio de tiempo la velocidad volverá a ser 0. En Turtlebot 3 la velocidad permanecerá constante. En el siguiente ejemplo podemos ver cómo mandar mensajes de este tipo desde Python:

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('teleoperacion')
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
while not rospy.is_shutdown():
    dato = input("Escribe un comando:(f:forward/l:left/r:right) ")
    cmd = Twist()
    if dato == 'f':
        cmd.linear.x = 0.25
    elif dato == 'l':
        cmd.angular.z = 0.75
        cmd.linear.x = 0.25  
    elif dato == 'r':
        cmd.angular.z = -0.75
        cmd.linear.x = 0.25        
    pub.publish(cmd)
```

podemos escribir una `f`, `l` o `r` (seguidas de INTRO) para hacer que el robot avance recto, hacia la izquierda o hacia la derecha una corta distancia.

Al igual que hicimos antes, como nuestro código depende de otro paquete: `geometry_msgs` deberíamos **añadir en el `package.xml` una etiqueta `<depend>`**indicándolo:

```xml
<depend>geometry_msgs</depend>
```

### Leyendo y publicando mensajes a la vez

Muchos nodos ROS serán a la vez productores y consumidores de mensajes. Tendrán por tanto un *callback* que recibe los mensajes a los que están suscritos, y dentro de él es donde normalmente publicarán sus propios mensajes. Por ejemplo, aquí tenemos la estructura que tendría un programa que recibiera los mensajes del laser y en función de los datos enviara otros mensajes para controlar los motores del robot:

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    #aquí haríamos los cálculos con las lecturas del laser
    #....
    #y en función de esos cálculos enviaríamos un comando de velocidad
    cmd = Twist()
    # cmd.linear.x = ...
    #cmd.angular.z = ...
    pub.publish(cmd)

rospy.init_node('productor_consumidor')
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
```
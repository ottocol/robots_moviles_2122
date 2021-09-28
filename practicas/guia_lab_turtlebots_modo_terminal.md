# Cómo trabajar con los robots turtlebot del laboratorio de robótica

Los turtlebot llevan a bordo un PC (Intel NUC) en el que está instalado ROS Indigo (excepto el número 5 que tiene Kinetic). Este ordenador tiene conexión wifi, aunque no tiene pantalla. Vamos a necesitar un PC adicional para poder interactuar con el robot.


## Conexión "física" con el robot

> Los PCs del laboratorio no tienen wifi, si estás usándolos pídele al profesor un pendrive wifi de los armarios de material 
 
Arranca el robot con el interruptor de la base. Espera un minuto a que arranque su ordenador de a bordo. Ahora tienes que elegir cuál de los dos métodos siguientes quieres usar para conectar con el robot:

- **Mediante VNC**: con la wifi podemos emplear los PCs del laboratorio (o tu portátil si lo prefieres) como terminales gráficos para el robot (es decir, básicamente como si fueran la pantalla/teclado/ratón del PC del robot). **No hace falta que vuestro PC tenga ROS** ni esté en linux ya que todo el código ROS se ejecuta en el propio robot.
- **Cliente/servidor**: En este modo casi todo el código ROS se ejecuta en nuestro PC y en el robot solo se ejecutan los nodos ROS que controlan el motor y los sensores **Vuestro PC debe tener instalado ROS**.

De momento, en las primeras prácticas vamos a usar el primer método, más adelante explicaremos el segundo.


## Conexión con el robot mediante VNC<a name="vnc"></a>

> Ten en cuenta que TODO el código ROS se ejecutará en el robot, y que el PC solo va a hacer de terminal (pantalla/teclado/ratón). Este método **no requiere que tengas instalado ROS en tu PC, ni siquiera linux**, y no ocupa demasiado ancho de banda. Como inconveniente, no podrás usar la herramienta `rviz`.

### Paso 1: Instalar en tu ordenador un cliente VNC
 
Conectaremos con el robot mediante VNC, que es un protocolo que permite conectar clientes gráficos (tu PC/el PC del lab) a servidores remotos (el robot). 

En cada robot turtlebot ya está funcionando un servidor VNC. Cualquier cliente VNC te debería permitir conectarte a los robots. Dependiendo del sistema operativo en el que estés tendrás que bajarte uno u otro:

- Para Windows puedes usar por ejemplo [VNC Viewer](https://www.realvnc.com/es/connect/download/viewer/windows/)
- En Ubuntu el cliente por defecto se llama **Remmina VNC**, puede que ya lo tengas instalado, si no, lo puedes instalar con el gestor de paquetes o la utilidad de instalación de software de Ubuntu.

### Paso 2: Conectarte a la wifi del laboratorio

Lo primero es conectarte a la red inalámbrica local del laboratorio. Hay 3 redes inalámbricas (`labrobot-wifi-5-1`, `labrobot-wifi-5-2`, `labrobot-wifi-2-4`) pero son todas la misma red local (hay 2 redes en 5 G y 1 en 2.4G para dispositivos inalámbricos más antiguos). Usa alguna de las que tengan un "5" en el nombre, serán más rápidas.

>
### Paso 3: Conectarte al robot

En tu programa cliente de VNC (VNC Viewer, Remmina,...) debes introducir la IP del robot al que te quieres conectar y la contraseña de conexión. Los datos son:

- Turtlebot 1. IP: 192.168.1.5 password: turtle_1 
- Turtlebot 2. IP: 192.168.1.6 password: turtle_2 
- Turtlebot 3. IP: 192.168.1.7 password: turtle_3 
- Turtlebot 4. IP: 192.168.1.8 password: turtle_4
- Turtlebot 5. IP: 192.168.1.9 password: turtle_5 

Debería aparecer la pantalla mostrando gráficamente el linux del robot.

### Paso 4: Arrancar ROS en el robot

Para que los sensores y servicios del robot empiecen a publicar datos en ROS:

```bash
#arranque "mínimo" para que la base se mueva
roslaunch turtlebot_bringup minimal.launch
#si necesitas el laser
roslaunch turtlebot_bringup hokuyo_ust10lx.launch
#si necesitas la cámara RGBD
roslaunch astra_launch astra.launch
```

> NO es necesario arrancar siempre todos los servicios y sensores, es mejor arrancar los mínimos imprescindibles. Por ejemplo para la práctica 1 no necesitas la cámara RGBD (`astra`), solo el laser, por lo que puedes obviar la última línea

Tras la ejecución de estos comandos el robot estará operativo, publicando información sobre su odometría, sensores de contacto, laser, etc. Para comprobarlo prueba a consultar los topics disponibles, a ver si aparece una lista. Comprueba que aparezca `/scan`, (el laser) que es el que necesitas para la práctica 1

```bash
rostopic list
```

también puedes probar a teleoperar el robot con el teclado para comprobar que se puede mover la base

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

### Copiar archivos entre el PC y el robot

> Como norma general: **NO BORRES NADA DEL ROBOT SALVO QUE LO HAYAS PUESTO TU, NO SOBREESCRIBAS NINGÚN WORKSPACE NI MODIFIQUES NINGÚN FICHERO DE CONFIGURACIÓN** 

Si solo quieres probar un código Python simple, en general **te bastará con copiar el fichero `.py`** (no hace falta copiar el *workspace* entero). Si son múltiples archivos también puedes copiar el workspace, pero ***NO LO LLAMES `catkin_ws`, EL ROBOT YA TIENE UN `catkin_ws` creado** 

Para las operaciones de copia de archivos se usará el usuario `turtlebot`, cuya contraseña es `ros`, excepto para el turtlebot número 5, cuyo usuario es `tb2`, no `turtlebot`.


### Desde Windows

Puedes usar una aplicación llamada [WinSCP](https://winscp.net/eng/docs/lang:es)

### Desde Linux

La mayoría de administradores de archivos de Linux pueden mostrar directorios remotos, prueba a ir al administrador de archivos y en la barra de direcciones a escribir: `ssh://turtlebot@192.168.1.5` **cambia por la IP del robot que tengas, y recuerda que el usuario del nº5 es `tb2` no `turtlebot`**. Te debería pedir la contraseña ("ros") y mostrar el contenido de la carpeta *home* del robot.

Si el administrador de archivos no funciona, puedes también teclear los comandos en la terminal, aunque será algo más tedioso: 

Copiar desde el PC al robot:

```bash
#Esto se teclea DESDE UNA TERMINAL DEL LINUX del PC, NO en el VNC
scp mi_archivo.zip turtlebot@IP_del_robot:~
```

te pedirá la contraseña para el usuario (que en nuestro caso es `ros`) y lo copiará al directorio `home` de ese usuario.

Copiar desde el robot al PC:

```bash
#Esto se teclea DESDE UNA TERMINAL DEL LINUX del PC, NO en el VNC
#FIJATE EN EL '.' DEL FINAL, esto hace que se copie en la carpeta actual
scp turtlebot@IP_del_robot:~/mi_archivo.zip .
```


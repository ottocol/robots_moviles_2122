# Práctica evaluable 1: moverse evitando obstáculos

## Objetivo

**Desarrollad un programa en ROS que haga que el robot se mueva por el entorno evitando los obstáculos**.  No tiene por qué tener un destino determinado, simplemente "vagabundear" por el entorno. Aunque en la asignatura veremos algoritmos de evitación de obstáculos bastante sofisticados, no se pide nada de eso ya que el objetivo es simplemente que os familiaricéis con la programación en ROS.

Como idea sencilla podéis mirar las lecturas que apuntan más o menos a la derecha y las que apuntan más o menos a la izquierda (en lugar de tomar una sola podéis coger la media de varias que estén en un ángulo similar). Si las lecturas de la izquierda son menores que las de la derecha hay que girar hacia la derecha, y viceversa. Podéis añadir mejoras, como por ejemplo:

 - Que la velocidad de giro sea mayor si la diferencia entre izquierda y derecha es mayor
 - Que la velocidad lineal sea proporcional a la distancia al obstáculo más cercano º
 - ...cualquier otra idea que se os ocurra
 
 **IMPORTANTE** una vez entregada la práctica la podréis probar con los Turtlebot 2 del laboratorio, por lo que sería interesante que la probárais antes en el simulador de Turtlebot 2 si tenéis la posibilidad

## Ayuda para la implementación: sensores de rango 2D en ROS

Los sensores de rango 2D dan distancias a obstáculos en un rango angular determinado, normalmente los sensores apuntan hacia donde está mirando el robot. Podéis obtener la información del sensor normalmente en el topic `/scan` o `/base_scan`, dependiendo del robot o del simulador usado. Probad a ver cuál aparece al hacer un `rostopic list`.

Los mensajes en este *topic* son del tipo `sensor_msgs/LaserScan`. El campo más importante de estos mensajes es el array `ranges`, pero además hay otros campos importantes. Podéis consultarlos por ejemplo en [http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/LaserScan.html](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/LaserScan.html). 

Dos campos interesantes son `angle_min` y `angle_max`, que como indica la página anterior son el ángulo donde empieza el scan y donde acaba. Teniendo en cuenta además el campo `angle_increment` podemos saber en qué ángulo apunta cada lectura del array ranges. La [0] apuntará a `angle_min`, la [1] a `angle_min+angle_increment`, y así sucesivamente.

Llevad cuidado porque en ROS el sentido de giro positivo es "a izquierdas" y eso da lugar a algunas consecuencias un poco antiintuitivas.

Al menos en los sensores simulados en Stage y en Gazebo con el Turtlebot 2, las primeras posiciones del array apuntan "a la derecha" del robot visto desde su punto de vista (ángulos negativos), y las últimas "a la izquierda" (ángulos positivos). El punto medio del array serían 0 radianes, la dirección en la que mira el robot. Por ejemplo en el scan del Turtlebot en gazebo va de -0,52 radianes aprox. a 0.52.

Esto es así para ser coherente con el sistema de coordenadas de ROS: a mayor ángulo "más giras a la izquierda" (en [https://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/](https://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/) tenéis una figura que lo explica mejor). 

En conclusión: para saber el ángulo de una lectura tenéis que hacer cuentas con `angle_min`, `angle_max` y `angle_increment`. 

En los Turtlebot 3 simulados en Gazebo las lecturas empiezan con angle_min=0 (al frente del robot) y terminan en 6.28 radianes (dan una visión de 360 grados alrededor del robot).

## Normas de entrega

La práctica se puede desarrollar **por parejas o bien individualmente**. 

> Podéis compartir con vuestros compañeros las ideas de cómo funciona el algoritmo (y de hecho si lo discutís y os dais ideas entre vosotros será más enriquecedor) pero el código Python o C++ debería ser exclusivamente vuestro. Sería recomendable que si habéis colaborado en el desarrollo del algoritmo os referenciéis unos a otros en la documentación entregada ("la idea del algoritmo ha surgido en colaboración con XXX e YYY...")

La práctica se entregará a través de moodle. El plazo de entrega concluye el **martes 19 de octubre a las 23:59**. Si la hacéis por parejas solo debéis hacer una entrega conjunta desde el moodle de uno de los dos componentes. En la documentación figurará el nombre de ambas personas.

Tenéis que entregar:

- **Código fuente** del programa con comentarios de cómo funciona
- **Documentación de las pruebas realizadas**: en qué circunstancias funciona el algoritmo, en cuáles ha fallado, por qué creéis que lo ha hecho y cómo creéis que se podría solucionar. Podéis incluir capturas de pantalla, adjuntar videos, los ficheros de los mundos...
    + Como mínimo probad un mundo en el simulador stage (usando el `ejemplo.world` que vimos en la introducción a ROS), y otro en el simulador gazebo. Ten en cuenta que el sensor simulado en stage (un láser) tiene mucho más campo de visión que el de Gazebo (una cámara 3D): 270 grados para el láser vs. 60 para la cámara, y eso influirá en el comportamiento del algoritmo, más que el que sea un entorno 2D o 3D.
  
**IMPORTANTE**: en evitación de obstáculos (como en todo lo demás) **no hay algoritmos perfectos  ni que funcionen siempre** igual de bien en todos los casos. No debéis intentar "esconder" los casos en los que vuestro código no funciona, sino documentarlos, intentar encontrarle una explicación y (idealmente) proponer mejoras que lo solucionarían. Para poder aplicar un algoritmo es importante conocer sus limitaciones.

## Baremo de evaluación

- **Hasta un 6**: código entregado y documentado (con comentarios al fuente) y una explicación breve (de 1/2 a 1 página) de la idea básica de vuestro algoritmo
- **Hasta un 7**: todo lo anterior más 2 pruebas documentadas: una con un sensor con un campo de visión pequeño (menor de 90 grados) y otro con un campo de visión amplio (mayor o igual a 270 grados). Podéis cambiar los parámetros del algoritmo o algo del código de un caso a otro.
    + Para el primer caso (sensor de campo de visión pequeño) podéis usar el simulador del Turtlebot 2, que simula un sensir de rango con Kinect. O podéis modificar el `ejemplo.world` de Stage que usamos en la primera sesión para disminuir su campo de visión (línea 5 del `ejemplo.world`, el valor del `fov` está en grados, cambiarlo por 60 por ejemplo para simular el *fov* de una kinect)
    + Para el segundo caso podéis usar el `ejemplo.world` tal cual está o bien las simulaciones de Turtlebot 3, que simulan un campo de visión de 360 grados
- **Hasta un 8**: todo lo anterior más pruebas documentadas en al menos otro entorno de stage y otro de gazebo, que sean diferentes a los anteriores (p.ej. pocos obstáculos vs muchos). Podéis buscar entornos por internet o crearlos vosotros.
- **Hasta un 10**: todo lo anterior más hacer que el robot en lugar de "vagabundear" sin rumbo intente ir a un destino que se especificará como una coordenada `(x,y)` en el sistema de coordenadas `odom`. El programa debería admitir como argumentos estas coordenadas.
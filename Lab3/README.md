# Carlos Felipe Sánchez & Felipe Cruz
# Universidad Nacional de Colombia
# Robótica - 2023-II
# Laboratorio 3
![](./Imgs/ESCUDO.png)

# Contenido
En el repositorio de este laboratorio se encuentra lo siguiente:
- README.md -> Archivo base con la descripción del laboratorio.
- Python -> Carpeta del catkin workspace. Contiene todo el proyecto de ROS para Python.
- Matlab -> Carpeta que contiene funciones para trabajar con ROS y un script de ejemplo.
- Imgs -> Carpeta con imágenes utilizadas en el archivo README.
- Videos -> Carpeta con los videos resultantes del desarrollo de la práctica.

---
Tabla de Contenidos
---

- [1. Introducción](#1-introducción)
- [2. Metodología](#2-metodología)
- [3. Código Matlab](#3-código-matlab)
- [4. Código Python](#4-código-python)
- [5. Resultados (videos) y Análisis](#5-resultados-videos-y-análisis)
- [6. Conclusiones](#6-conclusiones)

# 1. Introducción

En el presente laboratorio se presenta la prueba de conocimiento y funcionalidad del software ROS, implementado en un Sístema Operativo de código abierto compatible, y la prueba de conexión con los entornos de programación de MATLAB y Python.
- **Qué es ROS.**

ROS(Robot Operating System) es un entorno de desarrollo para software orientado a la robótica, este entorno funciona similar a un Sistema Operativo, permite crear conexiones y relaciones entre múltiples elementos robóticos de manera eficaz para simplificar su implementación. Provee control de dispositivos, simulación de entornos físicos, administración del Hardware, creación de directrices y relaciones maestro-esclavo, implementación de funcionalidad de uso común, comunicación entre los dispositivos, y mantenimiento de paquetes.
- **Por qué usamos ROS.**

Para poder trabajar en el medio robótico moderno, es necesario tomar ventaja de todas las herramientas a nuestro alcance, ROS nos provee de un medio de desarrollo mantenido por su comunidad, lo que nos permite desarrollar basados en trabajo establecido para alcanzar metas por medios más eficaces sin necesidad de tener que desarrollar un entorno o un medio de conexión desde ceros, además de poder apoyarnos en esta comunidad en caso de que se llegue a algún eprcance durante el proceso de desarrollo.

# 2. Metodología
- **Equipo.**

Se requiere un PC capaz de soportar Ubuntu, ya sea mediante la implementación de una máquina virtual o la instalación de Ubuntu en la raíz de la máquina, se requiere una imagen de Ubuntu 20.04.xxx, puesto que esta es la última versión del SO compatible con ROS a conocimiento presente, versión reciente de Python con la librería rospy, MATLAB 2015+ con el Robotic Toolbox y espacio en disco suficiente para instalar y testear el software.
- **Procedimiento**
 
Con Linux operando lanzar 2 terminales.
En la primera terminal escribir el comando roscore para iniciar el nodo maestro.
En la segunda terminal escribir rosrun turtlesim turtlesim node.
* Sección de MATLAB

Lanzar una instancia de Matlab para Linux
Crear un script con el siguiente código:
```
%%
rosinit; %Conexión con nodo maestro
%%
velPub = rospublisher(’/turtle1/cmd_vel’,’geometry_msgs/Twist’); %Creaci´on publicador
velMsg = rosmessage(velPub); %Creaci´on de mensaje
%%
velMsg.Linear.X = 1; %Valor del mensaje
send(velPub,velMsg); %Envío
pause(1)
```
Ejecutar las tres secciones del script y observar los resultados con la pose de la tortuga.

Crear un script en Matlab que permita suscribirse al tópico de pose de la simulación de turtle1.
Crear un script en Matlab que permita enviar todos los valores asociados a la pose de turtle1.
* Sección de Python

En el paquete hello turtle de ROS, en la carpeta de scripts, crear un script de Python, de nombre myTeleopKey.py
Escribir un código que permita operar una tortuga del paquete turtlesim con el teclado, que cumpla con las siguientes especificaciones:
• Se debe mover hacia adelante y hacia atrás con las teclas W y S
• Debe girar en sentido horario y antihorario con las teclas D y A.
• Debe retornar a su posición y orientación centrales con la tecla R
• Debe dar un giro de 180° con la tecla ESPACIO

Incluir el script que se acaba de crear en el apartado de catkin install python del archivo CMakeLists.txt, siguiendo la misma estructura de los otros scripts ya incluidos.
Lanzar una terminal, dirigirse al directorio del workspace de catkin y escribir el comando catkin make para hacer build en el paquete modificado.
Con Linux operando lanzar 3 terminales. En la primera terminal escribir el comando roscore para iniciar el nodo maestro.
En la segunda terminal escribir rosrun turtlesim turtlesim node.
En la tercera terminal dirigirse al directorio que contiene el workspace de catkin y escribir source devel/setup.bash. Acto seguido escribir rosrun hello turtle myTeleopKey.py. En este punto, la terminal ya deber´ıa estar esperando el ingreso de teclas.
Observar el movimiento de la tortuga con las teclas A, S, W y D, así como los cambios en la posición instantáneos con las teclas R y ESPACIO.
- **Recopilación de datos**

# 3. Código Matlab
A continuación se presenta cada una de las funciones creadas en Matlab junto con una explicación de qué hace y cómo se usa.

**cleanTurtle**
```
function [] = cleanTurtle()
    clear;
    clc;

    % Reiniciar Turtle
    service_name = '/reset';
    service_type = 'std_srvs/Empty';
    client = rossvcclient(service_name, service_type);
    req = rosmessage(service_type);
    call(client, req);
end
```
Esta función limpia las variables y la consola de Matlab. Luego, utiliza el servicio *Empty* para devolver la simulación de la tortuga a su estado inicial.

**connectROS**
```
function [] = connectROS()
    rosinit('http://ubuntu:11311/');
end
```
Esta función inicializa ROS en Matlab. Funciona desde Matlab en un computador Windows 10, que esté corriendo una terminal de ROS sobre Ubuntu 20 en una máquina virtual VMWare Player.

**disconnectROS**
```
function [] = disconnectROS()
    rosshutdown;
end
```
Esta función finaliza la conexión con ROS en Matlab.

**initTurtleROS**
```
function [posePub, poseSub] = initTurtleROS()
    posePub = publishROS('/turtle1/cmd_vel','geometry_msgs/Twist');
    poseSub = subscribeROS('/turtle1/pose','turtlesim/Pose');
end
```
Esta función crea un publisher y un subscriber de ROS para leer y escribir sobre la pose de la simulación de la tortuga, usando funciones personalizadas que se describen más adelante. La función devuelve los objetos publisher y subscriber para usarse en otras partes de Matlab.

**publishROS**
```
function [pub] = publishROS(topicName,messageType)
    pub = rospublisher(topicName,messageType);
end
```
Esta función crea un publisher de ROS. Recibe topic y tipo de mensaje, y devuelve el objeto publisher. En la función *initTurtleROS* se usa.

**subscribeROS**
```
function [sub] = subscribeROS(topicName,messageType)    
    % Crea el suscriptor
    sub = rossubscriber(topicName, messageType);
end
```
Esta función crea un subscriber de ROS. Recibe topic y tipo de mensaje, y devuelve el objeto subscriber. En la función *initTurtleROS* se usa.

**readPoseROS**
```
function [x, y, a] = readPoseROS(sub)
    % Obtener el último mensaje
    poseMsg = sub.LatestMessage;
    
    % Accede a los datos del mensaje
    if ~isempty(poseMsg)
        x = poseMsg.X;
        y = poseMsg.Y;
        a = poseMsg.Theta;
        disp("x: " + x + " y: " + y + " theta: " + a);
    else
        x = nan;
        y = nan;
        a = nan;
        disp("x: " + x + " y: " + y + " theta: " + a);
    end
end
```
Esta función recibe un objeto del tipo subscriber y devuelve la pose de la tortuga. También imprime en pantalla los valores de la pose. En caso de que no se haya recibido un primer mensaje, *LatestMessage* estará vacío y se colocarán los valores en NaN.

**writePoseROS**
```
function [] = writePoseROS(pub,x,y,a)
    % Crear Mensaje
    msg = rosmessage(pub);

    % Editar Mensaje
    if ~isnan(x)
        msg.Linear.X = x;
    end
    if ~isnan(y)
        msg.Linear.Y = y;
    end
    if ~isnan(a)
        msg.Angular.Z = a;
    end

    % Enviar Mensaje
    send(pub,msg);
end
```
Esta función recibe un objeto del tipo publisher, y los valores para la pose *x, y,* y *a* (theta). La función crea el mensaje teniendo en cuenta lo recibido, y lo publica en el topic del publisher. La tortuga se moverá lo descrito en base a su posición actual. Para solo mover en uno de los valores, basta con mandar los demás en 0 o en NaN.

**teleportPoseROS**
```
function [] = teleportPoseROS(x,y,a)
    service_name = '/turtle1/teleport_absolute';
    service_type = 'turtlesim/TeleportAbsolute';
    
    % Crear un cliente de servicio
    client = rossvcclient(service_name, service_type);
    
    % Crear un mensaje de solicitud
    msgServ = rosmessage(service_type);
    
    % Establecer la posición y orientación a la que deseas teleportar
    msgServ.X = x; 
    msgServ.Y = y;   
    msgServ.Theta = a;
    
    % Llamar al servicio para teleportar
    call(client, msgServ);
end
```
Esta función recibe los valores para la pose *x, y,* y *a* (theta). La función utiliza el servicio de teleportación absoluta para forzar la pose de la tortuga a la pose especificada, de manera inmediata.

Finalmente, se utilizan todas las funciones anteriores en un único script de ejemplo.
```
% Conectar a ROS
connectROS();

% Limpiar Matlab y Turtle
cleanTurtle();

% Inicializar turtle
[posePub, poseSub] = initTurtleROS();
pause(10);

% Mover turtle
writePoseROS(posePub,1,0,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,0,1,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,0,0,pi/2);
pause(1);
writePoseROS(posePub,1,0,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,0,1,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,1,1,pi);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,1,0,0);
pause(1);
readPoseROS(poseSub);
writePoseROS(posePub,0,2,0);
pause(1);
readPoseROS(poseSub);
teleportPoseROS(0,0,0);
pause(1);
readPoseROS(poseSub);

% Desconectar ROS
disconnectROS();
```
En la sección de Resultados se puede observar el video del funcionamiento del script. Este script realiza lo siguiente usando las funciones:
1. Se conecta con ROS.
2. Reinicia la simulación de la tortuga y las variables de Matlab.
3. Inicializa el publisher y subscriber para trabajar con la pose de la tortuga.
4. Realiza varios movimientos (y al final una teleportación). Luego de cada movimiento lee la nueva pose.
5. Se desconecta de ROS.

# 4. Código Python
Para el proyecto de ROS en Python, se utiliza como base el repositorio: https://github.com/felipeg17/hello_turtle.
El repositorio se incluye dentro de la carpeta que tiene el *catkin_workspace* para poderlo compilar.
La práctica consiste en el desarrollo de un script que permita el control de la tortuga desde el teclado. 
1. **W** para mover hacia adelante.
2. **S** para mover hacia atrás.
3. **A** para rotar hacia la izquierda de la tortuga.
4. **D** para rotar hacia la derecha de la tortuga.
5. **R** para teleportar la tortuga a su pose inicial.
6. **SPACE** para rotar la tortuga 180 grados.

A continuación, se presenta la función *myTeleopKey.py*, correspondiente al script requerido. Los comentarios dentro del código y el nombre de las variables y funciones, explican el funcionamiento del código.
```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from pynput import keyboard
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import TeleportRelative
from numpy import pi

# Variables globales
turtlePose = None
turtleVelLinear = 1
turtleVelAngular = 30 * pi / 180.0

def readPose(pose):
    # Actualizar pose. Se ejecuta por el subscriber.
    global turtlePose
    turtlePose = pose
    print("x: " + str(turtlePose.x) + " y: " + str(turtlePose.y) + " theta: " + str(turtlePose.theta * 180.0 / pi))

def onPress(key):
    try:
        # Leer tecla presionada
        if key == keyboard.Key.space:
            key = " "
        else:
            key = key.char
    except AttributeError:
        # Ignorar las teclas diferentes a caracteres y ESPACIO
        return

    # Mover en base a la tecla presionada
    if key in ["w", "a", "s", "d", "r", " "]:
        if key == "r":
            # Reiniciar tortuga
            resetTurtle()
        elif key == " ":
            # Girar tortuga 180 grados
            turnTurtle()
        else:
            # Usar Twist para mover a la tortuga
            twist = Twist()
            twist.linear.y = 0

            # Cambiar movimiento en base a la tecla presionada
            if key == "w":
                twist.linear.x = turtleVelLinear
                twist.angular.z = 0
            elif key == "s":
                twist.linear.x = -turtleVelLinear
                twist.angular.z = 0
            elif key == "a":
                twist.linear.x = 0
                twist.angular.z = turtleVelAngular
            elif key == "d":
                twist.linear.x = 0
                twist.angular.z = -turtleVelAngular
            
            # Publicar movimiento de la tortuga
            pub.publish(twist)

def main():
    # Inicializar nodo
    rospy.init_node('turtle_keyboard_control')

    # Definir publisher
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

    # Definir subscriber
    rospy.Subscriber('turtle1/pose', Pose, readPose)
    rate = rospy.Rate(10)

    # Leer teclado
    with keyboard.Listener(on_press=onPress):
        while not rospy.is_shutdown():
            rate.sleep()

def resetTurtle():
    try:
        # Usar servicio de teleport absolute
        teleport_absolute = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        new_pose = Pose()
        new_pose.x = 5.5 # Default x
        new_pose.y = 5.5 # Default y
        new_pose.theta = 0.0 # Default theta
        teleport_absolute(new_pose.x, new_pose.y, new_pose.theta)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def turnTurtle():
    try:
        # Usar servicio de teleport absolute
        teleport_absolute = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        new_pose = Pose()
        new_pose.x = turtlePose.x # Mantener x
        new_pose.y = turtlePose.y # Mantener y
        new_pose.theta = turtlePose.theta + pi # Rotar pi radianes (180 grados)
        teleport_absolute(new_pose.x, new_pose.y, new_pose.theta)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    main()

```
En la sección de Resultados, se observa en un video cómo ejecutar el script anterior desde la terminal y su funcionamiento.
# 5. Resultados (videos) y Análisis

https://github.com/cafsanchezdi/Robotica2023-2/assets/45526932/f3420785-a2e6-43ef-89b2-3d2f4c4006eb

https://github.com/cafsanchezdi/Robotica2023-2/assets/45526932/25c8c284-a289-4d10-a634-c3075d46dd44

# 6. Conclusiones




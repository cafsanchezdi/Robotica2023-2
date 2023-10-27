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
# 2. Metodología
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
# 6. Conclusiones

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

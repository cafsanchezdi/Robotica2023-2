"""
Allows to use the service dynamixel_command 
"""
import rospy
import time
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
import tkinter as tk
from PIL import Image, ImageTk
from sensor_msgs.msg import JointState
import math
import numpy as np
import random

__author__ = "Felipe Cruz"
__credits__ = ["Felipe Cruz"]
__email__ = "fcruzv@unal.edu.co"
__status__ = "Test"

# Parametros
L1 = 44.5
L2 = 106
L3 = 105
L4 = 75
TorqueMax = 1023

# Estado del robot
ToolCargada = False
Rutina = None
TiempoInicial = time.time()
TiempoRutina = 0

# Cinematica inversa
def getJointAngles(x,y,z):
    q1 = math.atan2(x,y)
    w = np.array([x,y,z]) - L4 * np.array([x/np.sqrt(x**2+y**2),y/np.sqrt(x**2+y**2),0])
    r = np.sqrt(w[0]**2+w[1]**2)
    h = z - L1
    c = np.sqrt(r**2+h**2)
    phi = math.acos((c**2-L3**2-L2**2)/(-2*L2*L3))
    gamma = math.atan2(h,r)
    alpha = math.acos((L3**2-L2**2-c**2)/(-2*L2*c))
    q2 = np.pi / 2 - alpha - gamma
    q3 = np.pi / 2 - phi
    q4 = np.pi / 2 - q2 - q3
    toDeg = 180.0 / np.pi
    return q1*toDeg, q2*toDeg, q3*toDeg, q4*toDeg

# Funcion para mover el robot
def moveRobot(q1,q2,q3,q4,maxSpeed):
    moveJoint(1, castDegreesToGoal(q1), maxSpeed)
    moveJoint(2, castDegreesToGoal(q2), maxSpeed)
    moveJoint(3, castDegreesToGoal(q3), maxSpeed)
    moveJoint(4, castDegreesToGoal(q4), maxSpeed)

# Funcion para mover el robot con la inversa
def moveRobotInversa(x,y,z,maxSpeed):
    q1, q2, q3, q4 = getJointAngles(x,y,z)
    moveRobot(q1,q2,q3,q4,maxSpeed)

# Recoger Marcador
def RecogerMarcador():
    moveRobot(56,90,-90,90,100)
    abrirGriper(True)
    rospy.sleep(2)
    moveRobot(56,90,0,0,20)
    rospy.sleep(10)
    abrirGriper(False)
    rospy.sleep(2)
    moveRobot(56,0,0,0,20)
    rospy.sleep(2)
    moveRobot(0,0,0,0,100)
    rospy.sleep(2)

# Descargar Marcador
def DescargarMarcador():
    moveRobot(56,90,-90,90,50)
    rospy.sleep(4)
    moveRobot(56,90,0,0,20)
    rospy.sleep(10)
    abrirGriper(True)
    rospy.sleep(2)
    moveRobot(56,0,0,0,20)
    rospy.sleep(2)
    moveRobot(0,0,0,0,100)
    rospy.sleep(2)

# Dibujar Espacio
def DibujarEspacio():
    moveRobotInversa(-120,120,-85,15)
    rospy.sleep(15)
    moveJoint(1, castDegreesToGoal(45), 30)
    rospy.sleep(5)
    moveRobot(0,0,0,0,50) # Home

# Dibujar Letra
def DibujarLetra():
    moveRobotInversa(0,100,6,15)
    rospy.sleep(10)
    moveRobotInversa(10,100,2,15)
    rospy.sleep(3)
    moveRobotInversa(10,90,20,15)
    rospy.sleep(3)
    moveRobotInversa(0,90,20,15)
    rospy.sleep(3)
    moveRobotInversa(10,90,20,15)
    rospy.sleep(3)
    moveRobotInversa(10,70,39,15)
    rospy.sleep(3)
    moveRobot(0,0,0,0,50) # Home

# Dibujar Figura
def DibujarFigura():
    moveRobotInversa(-70,70,8,15)
    rospy.sleep(15)
    moveRobotInversa(-60,80,5,15)
    rospy.sleep(3)
    moveJoint(1, castDegreesToGoal(-10), 15)
    rospy.sleep(5)
    moveRobotInversa(-20,100,50,15)
    rospy.sleep(10)
    moveRobotInversa(100,100,-55,15)
    rospy.sleep(10)
    moveRobotInversa(70,100,-22,15)
    rospy.sleep(3)
    moveJoint(1, castDegreesToGoal(20), 15)
    rospy.sleep(5)
    moveRobot(0,0,0,0,50) # Home

# Crear la ventana de la interfaz gráfica
root = tk.Tk()
root.title("Control Phantom")

# Crear una variable de seguimiento para la selección de rutinas
rutina = tk.StringVar()

# Crear una función para mover los motores al punto seleccionado
def controlRobot():
    global ToolCargada
    global TiempoRutina
    Rutina = rutina.get()
    if Rutina == 'Recoger Marcador' and not ToolCargada:
        TiempoRutina = time.time()
        RecogerMarcador()
        TiempoRutina = time.time() - TiempoRutina
        ToolCargada = True
        return
    else:
        if not ToolCargada:
            return
        elif Rutina == 'Dibujar Espacio':
            TiempoRutina = time.time()
            DibujarEspacio()
            TiempoRutina = time.time() - TiempoRutina
            return
        elif Rutina == 'Dibujar Letra':
            TiempoRutina = time.time()
            DibujarLetra()
            TiempoRutina = time.time() - TiempoRutina
            return 
        elif Rutina == 'Dibujar Figura':
            TiempoRutina = time.time()
            DibujarFigura()
            TiempoRutina = time.time() - TiempoRutina
            return
        elif Rutina == 'Descargar Marcador':
            TiempoRutina = time.time()
            DescargarMarcador()
            TiempoRutina = time.time() - TiempoRutina
            ToolCargada = False
            return

# Crear una función para PARAR
def STOP():
    exit()

# Colocar nombres y titulo
tk.Label(root, text="HMI para controlar robot Phantom").pack()
tk.Label(root, text="Felipe Cruz").pack()
tk.Label(root, text="Carlos Sanchez").pack()

# Colocar logo de la U
image_path = '/home/feli/Documents/Robotica/Lab4/Catkin/src/scripts/UNAL.jpeg'
img = Image.open(image_path)
img = ImageTk.PhotoImage(img)
image_label = tk.Label(root, image=img)
image_label.pack()

# Crear etiqueta y menú desplegable para seleccionar un punto
point_label = tk.Label(root, text="Seleccionar una rutina:")
point_label.pack()
point_dropdown = tk.OptionMenu(root, rutina, *['Recoger Marcador','Dibujar Espacio', 'Dibujar Letra', 'Dibujar Figura', 'Descargar Marcador'])
point_dropdown.pack()

# Crear un botón para ejecutar comandos
move_button = tk.Button(root, text="Ejecutar", command=controlRobot)
move_button.pack()

# Crear espacios para mostrar info
time_label = tk.Label(root, text="Tiempo: 0")
time_label.pack()
rutina_label = tk.Label(root, text="Rutina: SIN DATOS...")
rutina_label.pack()
position_label = tk.Label(root, text="Pos: SIN DATOS...")
position_label.pack()
tool_label = tk.Label(root, text="Herramienta: SIN DATOS...")
tool_label.pack()

# Crear un botón para parar (de emergencia)
move_button = tk.Button(root, text="PARADA DE EMERGENCIA", command=STOP)
move_button.pack()

# Definir la función callback para actualizar los valores en tiempo real
def callback(data):
    toDeg = 180 / np.pi
    angulo1 = round(data.position[0] * toDeg, 2)
    angulo2 = round(data.position[1] * toDeg, 2)
    angulo3 = round(data.position[2] * toDeg, 2)
    angulo4 = round(data.position[3] * toDeg, 2)
    position_label.config(text=f"q1: {angulo1}\nq2: {angulo2}\nq3: {angulo3}\nq4: {angulo4}\n")

def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)

def update_gui():
    global ToolCargada
    global TiempoRutina
    global TiempoInicial
    listener() # Leer datos
    time_label.config(text=f"Tiempo Total: {round(time.time() - TiempoInicial,0)} segundos")
    rutina_label.config(text=f"Tiempo de Rutina: {round(TiempoRutina,2)} segundos")
    tool_label.config(text=f"Marcador: {'Cargado' if ToolCargada else 'Descargado'}")
    root.after(1000, update_gui)  # Actualizar

# Funcion para usar el servicio para mover los motores
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

# Funcion que mueve un motor a una posicion especifica
def moveJoint(id, val, maxSpeed):
    # id -> (1,4)
    # val -> (0,1023)
    if(not (id in [1,2,3,4])):
        raise RuntimeError("Wrong id!")
    jointCommand('', id, 'Torque_Limit', TorqueMax, 0)
    jointCommand('', id, 'Moving_Speed', maxSpeed, 0)
    jointCommand('', id, 'Goal_Position', max(min(1023,val),0), 0)

def abrirGriper(abierto):
    # abierto -> True o False
    jointCommand('', 5, 'Torque_Limit', 400, 0)
    jointCommand('', 5, 'Moving_Speed', 1023, 0)
    if(abierto):
        jointCommand('', 5, 'Goal_Position', 512, 0)
    else:
        jointCommand('', 5, 'Goal_Position', 170, 0)

# Convertir los grados al valor que recibe el motor
def castDegreesToGoal(deg):
    # deg -> (-150, 150)
    return round(3.41*deg + 511.5)

# Funcion principal. Manda al home y abre el HMI.
if __name__ == '__main__':
    try:   
        abrirGriper(True)
        moveRobot(0,0,0,0,50) # Home
    except rospy.ROSInterruptException:
        pass

    # Iniciar un hilo para la actualización de la GUI
    update_gui()

    root.mainloop() # HMI
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
import threading

__author__ = "Felipe Cruz"
__credits__ = ["Felipe Cruz"]
__email__ = "fcruzv@unal.edu.co"
__status__ = "Test"

# Definir los puntos objetivo
points = [
    [0,0,0,0],
    [25,25,20,-20],
    [-35,35,-30,30],
    [85,-20,55,25],
    [80,-35,55,-45]
]

# Crear la ventana de la interfaz gráfica
root = tk.Tk()
root.title("Control Phantom")

# Crear una variable de seguimiento para la selección de puntos
selected_point = tk.StringVar()

# Crear una función para mover los motores al punto seleccionado
def move_to_selected_point():
    point_index = int(selected_point.get())
    if 0 <= point_index < len(points):
        p = points[point_index]
        moveJoint(1, castDegreesToGoal(p[0]))
        moveJoint(2, castDegreesToGoal(p[1]))
        moveJoint(3, castDegreesToGoal(p[2]))
        moveJoint(4, castDegreesToGoal(p[3]))

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
point_label = tk.Label(root, text="Seleccionar un punto:")
point_label.pack()
point_dropdown = tk.OptionMenu(root, selected_point, *range(len(points)))
point_dropdown.pack()

# Crear un botón para mover a ese punto
move_button = tk.Button(root, text="Mover al punto", command=move_to_selected_point)
move_button.pack()

# Función para cerrar la ventana de la interfaz gráfica
def close_window():
    root.destroy()

# Crear un botón para cerrar la ventana
exit_button = tk.Button(root, text="Cerrar", command=close_window)
exit_button.pack()

# Crear espacio para mostrar la posicion de los motores
position_label = tk.Label(root, text="SIN DATOS...")
position_label.pack()

# Funcion para usar el servicio para mover los motores
def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

# Funcion que mueve un motor a una posicion especifica
def moveJoint(id, val):
    # id -> (1,4)
    # val -> (0,1023)
    if(not (id in [1,2,3,4])):
        raise RuntimeError("Wrong id!")
    jointCommand('', id, 'Torque_Limit', 400, 0)
    jointCommand('', id, 'Goal_Position', max(min(1023,val),0), 0.5)
    time.sleep(0.5)

# Convertir los grados al valor que recibe el motor
def castDegreesToGoal(deg):
    # deg -> (-150, 150)
    return round(3.41*deg + 511.5)

# Definir la función callback para actualizar los valores en tiempo real
def callback(data):
    position_label.config(text=f"{data.position}")

def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)

def update_gui():
    listener() # Leer datos
    root.after(1000, update_gui)  # Actualizar cada segundo

# Funcion principal. Manda al home y abre el HMI.
if __name__ == '__main__':
    try:   
        moveJoint(1,castDegreesToGoal(0))    
        moveJoint(2,castDegreesToGoal(0))    
        moveJoint(3,castDegreesToGoal(0))    
        moveJoint(4,castDegreesToGoal(0))   
        time.sleep(2)   
    except rospy.ROSInterruptException:
        pass

    # Iniciar un hilo para la actualización de la GUI
    update_gui()

    root.mainloop()
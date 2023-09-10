# Carlos Felipe Sánchez & Felipe Cruz
# Universidad Nacional de Colombia
# Robótica - 2023-II
# Laboratorio 1
![](./Imgs/ESCUDO.png)

# Contenido
En el repositorio de este laboratorio se encuentra lo siguiente:
- README.md -> Archivo base con la descripción del laboratorio.
- CAD -> Carpeta que contiene los modelos 3D de la herramienta diseñada en formatos .ipt, .iam, .stl, .step.
- RobotStudio -> Carpeta que contiene la solución de Robot Studio utilizada durante la práctica.
- Imgs -> Carpeta con imágenes utilizadas en el archivo README.
- Videos -> Carpeta con los videos resultantes del desarrollo de la práctica.

# Descripción de la solución planteada
La solución planteada se desarrolló siguiendo los siguientes pasos. En cada uno de los siguientes pasos se describe en detalle lo realizado.
## Diseño de la herramienta.
La herramienta del proceso corresponde a un marcador para escribir sobre papel. El diseño corresponde al diseño de un soporte que conecte el marcador con el flange del robot.

El diseño tuvo en consideración lo siguiente:
- El tamaño y forma del flange del robot real. El tamaño del flange y el tamaño y ubicación de los agujeros se obtiene del datasheet del robot en la página 61.

![](./Imgs/Flange.jpg)

*Plano del flange del robot. (Datasheet, pp 61).*

- El diámetro aproximado del marcador real a utilizar.
- La herramienta debe estar apoyada sobre el plano xy con el eje z hacia arriba (para facilitar la incorporación de esta dentro de RobotStudio).
- Colocación de un resorte para darle tolerancia al movimiento del robot.
- La punta del marcador no debe quedar alineada con el eje z ya que incrementa la probabilidad de que se presenten singularidades durante el movimiento del robot.

Teniendo en cuenta lo anterior, se realiza el diseño del soporte y el marcador en AutoDesk Inventor 2023. El resultado es el siguiente:

![](./Imgs/SoporteInventor.jpg)

*Soporte diseñado en Inventor. Note que la base se encuentra sobre el plano xy y la punta no se alinea con el eje z.*

![](./Imgs/EnsambleInventor.jpg)

*Herramienta diseñada y ensamblada en Inventor.*

Posteriormente, se utiliza la impresión 3D para manufacturar el soporte del marcador en PLA. Se utiliza un resorte común y silicona para obtener la herramienta final:

![](./Imgs/Herramienta.jpeg)


## Diseño de la figura a dibujar con el robot (incluye 5 letras de cada integrante y una decoración).

## Incorporación de la herramienta dentro de RobotStudio.
## Incorporación del WorkObject dentro de RobotStudio.
## Programación en RAPID de la trayectoria.
## Simulación del movimiento del robot en Robot Studio.
## Implementación de la solución en los robots reales.

# Diagrama de flujo de acciones del robot
# Plano de planta de la ubicación de cada uno de los elementos
# Descripción de las funciones utilizadas
# Código en RAPID del módulo utilizado para el desarrollo de la práctica
# Video que contenga la simulación en RobotStudio así como la implementación de la práctica con los robots reales



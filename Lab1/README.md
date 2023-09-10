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
## 1. Diseño de la herramienta.
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

*Fig1.1: Soporte diseñado en Inventor. Note que la base se encuentra sobre el plano xy y la punta no se alinea con el eje z.*

![](./Imgs/EnsambleInventor.jpg)

*Fig1.2: Herramienta diseñada y ensamblada en Inventor.*

Posteriormente, se utiliza la impresión 3D para manufacturar el soporte del marcador en PLA. Se utiliza un resorte común y silicona para obtener la herramienta final:

![](./Imgs/Herramienta.jpeg)

*Fig1.3: Herramienta final.*

## 2. Diseño de la figura a dibujar con el robot (incluye 5 letras de cada integrante y una decoración).

## 3. Incorporación de la herramienta dentro de RobotStudio.
Pasos de la incorporación:
1. El ensamble de la herramienta se exporta en formato .STEP.
2. Se importa el archivo en Robot Studio en "Importar geometría".
3. Se define un sistema de coordenadas sobre la punta de la herramienta con el eje z alineado al marcador.
4. Se crea la herramienta **Marcador** con la geometría y sistema coordenado definidos anteriormente.
5. Se asocia la herramienta con el manipulador de tal manera que la herramienta quede sobre el flange (articulación 6).

![](./Imgs/RS_Herramienta.jpg)

*Fig3.1: Herramienta definida en RobotStudio con su TCP.*

![](./Imgs/RS_Ensamble.jpg)

*Fig3.2: Herramienta ensamblada con el manipulador en RobotStudio.*

## 4. Definición del WorkObject dentro de RobotStudio.
## 5. Diseño de la trayectoria
## 6. Programación en RAPID de la trayectoria.
## 7. Simulación del movimiento del robot en Robot Studio.
## 8. Ubicación real de los elementos
## 9. Implementación de la solución en los robots reales.
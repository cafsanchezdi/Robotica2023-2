# Carlos Felipe Sánchez & Felipe Cruz
# Universidad Nacional de Colombia
# Robótica - 2023-II
# Laboratorio 4
![](./Imgs/ESCUDO.png)

# Contenido
En el repositorio de este laboratorio se encuentra lo siguiente:
- README.md -> Archivo base con la descripción del laboratorio.
- Catkin -> Carpeta del catkin workspace. Contiene todo el proyecto de ROS para Python.
- Matlab -> Carpeta que contiene un Matlab LiveScript para la simulación del robot con Peter Corke.
- Imgs -> Carpeta con imágenes utilizadas en el archivo README.
- Videos -> Carpeta con los videos resultantes del desarrollo de la práctica.

---
Tabla de Contenidos
---

- [1. Introducción](#1-introducción)
- [2. Código Matlab (Peter Corke)](#2-código-matlab-peter-corke)
- [3. Código Python (ROS)](#3-código-python-ros)
- [4. Configuración ROS](#4-configuración-ros)
- [5. HMI](#5-hmi)
- [6. Resultado Final](#6-resultado-final)
- [7. Robot vs Simulación](#7-robot-vs-simulación)
- [8. Conclusiones](#8-conclusiones)

# 1. Introducción

# 2. Código Matlab (Peter Corke)
![](./Imgs/DH.jpg)

*Fig2.1: Diagrama del Robot Phantom para obtener los parámetros DH y la simulación.*

Se realiza la medición de los eslabones del robot Phantom real y se obtienen los siguientes valores:
```
L1 = 0.0455; %m
L2 = 0.103; %m
L3 = 0.103; %m
L4 = 0.11; %m (TCP)
Lm = 0.0315; %m  
```

Se programa en Matlab la matriz DH del robot, con la cual se pueden definir los Links del objeto SerialLink de la librería Peter Corke:
```
DH = [q1	L1	0	            -pi/2	0	0;
      q2	0	sqrt(L2^2+Lm^2)	0	    0	-pi/2;
      q3	0	L3	            0	    0	0;
      q4	0	L4	            0	    0	0];

for i=1:size(DH)
    L(i) = Link(DH(i,:));
end

Phantom = SerialLink(L,'name','Phantom','tool',trotx(-pi/2)*troty(pi/2))
```

Se establecen los valores deseados para las articulaciones, los cuales son 5 puntos (incluyendo el Home -> punto 1):
```
target1 = deg2rad([0 0 0 0]);
target2 = deg2rad([25 25 20 -20]);
target3 = deg2rad([-35 35 -30 30]);
target4 = deg2rad([85 -20 55 25]);
target5 = deg2rad([80 -35 55 -45]);
```

Finalmente, con el siguiente código, se puede visualizar la pose del robot que se obtiene al fijar las articulaciones en los valores deseados:
```
figure();
Phantom = SerialLink(L,'name','Phantom','tool',trotx(-pi/2)*troty(pi/2));
view(...);
Phantom.plot(target_,'noa','jaxes','notiles','floorlevel',0,'noshadow','delay',1);
axis tight;
```

En el código anterior, el valor dentro de **view** cambia para cada pose (con el fin de visualizar mejor la posición), y dentro del **Phantom.plot** se debe colocar el target deseado.

Los resultados gráficos de cada una de las poses se encuentran en [7. Robot vs Simulación](#7-robot-vs-simulación).

# 3. Código Python (ROS)

# 4. Configuración ROS

# 5. HMI

# 6. Resultado Final

# 7. Robot vs Simulación

# 8. Conclusiones





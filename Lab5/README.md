# Carlos Felipe Sánchez & Felipe Cruz
# Universidad Nacional de Colombia
# Robótica - 2023-II
# Laboratorio 5
![](./Imgs/ESCUDO.png)

# Contenido
En el repositorio de este laboratorio se encuentra lo siguiente:
- README.md -> Archivo base con la descripción del laboratorio.
- Catkin -> Carpeta del catkin workspace. Contiene todo el proyecto de ROS para Python.
- Imgs -> Carpeta con imágenes utilizadas en el archivo README.
- Videos -> Carpeta con los videos resultantes del desarrollo de la práctica.

---
Tabla de Contenidos
---

- [1. Introducción](#1-introducción)
- [2. Descripción de la Solución](#2-descripción-de-la-solución)
- [3. Base Portaherramientas y Marcador](#3-base-portaherramientas-y-marcador)
- [4. Cinemática Inversa](#4-cinemática-inversa)
- [5. Código](#5-código)
- [6. HMI](#6-hmi)
- [7. Videos](#7-videos)
- [8. Conclusiones](#8-conclusiones)

# 1. Introducción

El proyecto se centró en el desarrollo de una interfaz humano-máquina (HMI) para el control de un robot Phantom x100, también conocido como Pincher, de 4 grados de libertad. El objetivo principal consistía en habilitar la ejecución de cinco rutinas específicas: recoger marcador, dibujar el espacio de trabajo, representar letras, crear figuras y descargar el marcador.

Los robots Phantom, reconocidos por su versatilidad y aplicación en entornos educativos, son manipuladores de bajo costo y alta versatilidad. Estos robots, habitualmente compuestos por servomotores Dynamixel, ofrecen una estructura modular y son fácilmente programables. Son ampliamente utilizados en entornos educativos y de investigación para enseñar conceptos de robótica, cinemática y control de robots.

El proceso de creación de esta solución involucró múltiples etapas, desde el diseño de un soporte para el marcador mediante impresión 3D, la obtención de las fórmulas de cinemática inversa para el robot, hasta la implementación de ROS y Dynamixel Workbench en Python para el control del robot y la programación de las rutinas asociadas. Finalmente, se logró una integración exitosa entre la ejecución de las rutinas y la HMI desarrollada.

Los resultados obtenidos reflejaron un funcionamiento adecuado de todas las etapas del proyecto, a pesar del desafío inherente en el proceso de desarrollo de las rutinas, el cual se basó en ajustes iterativos debido a la incertidumbre generada por las mediciones reales del robot.

El presente informe documenta detalladamente cada fase del desarrollo, destacando los aspectos clave y los logros alcanzados en la implementación de la solución propuesta.

![](./Imgs/Phantom.jpg)

*Fig1.1: Robot Phantom.*

# 2. Descripción de la Solución

## Etapa 1: Diseño del Soporte para el Marcador en Impresión 3D
Se creó un soporte específico para el marcador utilizando tecnología de impresión 3D. Los detalles del diseño se encuentran en la [sección 3](#3-base-portaherramientas-y-marcador).

## Etapa 2: Obtención de Fórmulas para Cinemática Inversa
Se procedió a obtener las fórmulas necesarias para la cinemática inversa del robot Phantom x100. Estas fórmulas se basaron en mediciones de dimensiones del robot real, como se describe en la [sección 4](#4-cinemática-inversa).

## Etapa 3: Implementación de ROS y Dynamixel Workbench en Python
Se utilizó ROS junto con el entorno de trabajo de Dynamixel, todo implementado en Python, para la conexión y control del robot. Esta etapa permitió la interacción fluida con el robot.

## Etapa 4: Programación de Rutinas Utilizando Cinemática Inversa
Las rutinas requeridas se programaron en Python, involucrando el uso de la cinemática inversa para puntos específicos del robot. Los detalles de esta implementación se encuentran en la [sección 5](#5-código).

## Etapa 5: Integración de Rutinas con la HMI Desarrollada
Se logró la integración entre la ejecución de las rutinas y la interfaz humano-máquina (HMI) desarrollada. Esta conexión, también implementada en Python, permitió la interacción intuitiva del usuario con el robot. Se detalla en la [sección 6](#6-hmi).

## Nota Importante
El proceso de desarrollo de las rutinas se basó en un enfoque de prueba y error debido a la incertidumbre inherente en el modelo de cinemática inversa, originada por las mediciones reales del robot. Esto permitió ajustes iterativos para mejorar la precisión y funcionalidad de las rutinas.

# 3. Base Portaherramientas y Marcador

Para facilitar la ejecución precisa de las rutinas, se diseñó y fabricó un soporte especial para el marcador del robot Phantom x100. El proceso de creación de este soporte se dividió en dos etapas fundamentales: modelado en CAD e impresión 3D.

## Diseño en CAD y Características del Soporte
El diseño se realizó meticulosamente utilizando herramientas de modelado en CAD, con un enfoque en la funcionalidad y la no interferencia con el tablero de trabajo. Se priorizó la necesidad de asegurar el soporte al tablero de manera estable y segura, evitando dañar el tablero y permitiendo una colocación y extracción sencilla del marcador.

## Impresión 3D y Resultado Final
Tras completar el diseño, se procedió a la fabricación del soporte mediante impresión 3D. El resultado fue un soporte ergonómico y funcional que se ajusta adecuadamente al robot Phantom x100. Las siguientes imágenes muestran el resultado final:

![](./Imgs/Soporte.jpg)

*Fig3.1: Soporte para el marcador.*

La pieza pequeña es para ajustar el soporte al tablero con tornillos M6, de tal manera que los tornillos no tengan contacto con el tablero y así evitar daños.

![](./Imgs/SoporteMontado.jpg)

*Fig3.2: Soporte para el marcador colocado sobre el tablero. Se ajusta con tornillos M6.*

![](./Imgs/SoporteMarcador.jpg)

*Fig3.3: Soporte con el marcador.*

El marcador se coloca sin tapa sobre el soporte mirando hacia abajo. El agujero donde se encuentra el marcador tiene esa pequeña curvatura para facilitar la inserción del marcador.

El diseño y fabricación de este soporte jugaron un papel fundamental en el correcto funcionamiento de las rutinas programadas, permitiendo una manipulación precisa y segura del marcador durante la ejecución de las tareas específicas del proyecto.

## Nota:
Este soporte fue diseñado por: https://github.com/anhernadezdu/Laboratorio5_Cinematica-Inversa_PhantomX_ROS.

Nosotros realizamos la impresión 3D de ambos portaherramientas. En este caso, debido a la baja resolución que se requiere y el hecho de que el portaherramientas no requiere soportar cargas pesadas, se realizó una impresión a alta velocidad y con bajo relleno para optimizar tiempos y material.

# 4. Cinemática Inversa
El desarrollo de la cinemática inversa se hizo en base al desarrollo que se realiza en el siguiente proyecto:
https://github.com/cychitivav/px100_ikine

# 5. Código

# 6. HMI

# 7. Videos

# 8. Conclusiones
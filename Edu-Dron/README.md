# Edu-dron AirSim version
Es un proyecto creado para simplificar a los educadores de robótica el uso de AirSim con el simulador de dron.

Acceso a videos en Youtube:

 1.- Introducción: https://www.youtube.com/watch?v=7ywabVjzQF0
 
 2.- Instalación entornos AirSim : https://youtu.be/k0awHHWqkUo
 
 3.- Instalación Python : https://youtu.be/aQCRuMWiV_Q
 
 4.- Instalación AirSim + Requerimientos: https://youtu.be/igjjwGFyvfM
 
 5.- Instalación Edu-Dron: https://youtu.be/-SPRkelX0rM
 
 6.- Ejecución entorno Edu-Dron (desde Terminal) : https://youtu.be/JjvKoA8DGao
 
 7.- Instalación Visual Studio Code + Python : https://youtu.be/J3JcXuviMUI
 
 8.- Ejecución Edu-Dron desde Visual Studio Code: https://youtu.be/kHtzcnzvgOw



### Airsim
[Airsim](https://github.com/microsoft/AirSim) es un simulador para coches y drones está basado en [Unreal Engine](https://www.unrealengine.com/).
Es de código abierto, multiplataforma con simulaciones realistas física y visualmente.
Está desarrollado como un plugin de Unreal y por lo tanto se puede colocar en cualquier entorno de Unreal.

No se requieren prácticamente conocimientos técnicos (nivel usuario) para poner en marcha la formación a nivel de primaria o secundaria,
y puede ir haciéndose más complejo según necesitemos.

Es software libre y puede usarse sin limitaciones.

## Puesta en marcha
### Instalación
Está pensado para ejecutar en Microsoft Windows aunque podrían usarse en otros entornos.

Sigue las instrucciones del 'Manual del profesor Edu-Dron.pdf' y visualiza los pasos en el video de
Youtube.



Direcciones de las descargas: 
1.- Descarga Entornos AirSim: https://github.com/Microsoft/AirSim/releases 

2.- Direcciones descarga AirSim y Requerimientos: 

    Airsim : https://pypi.org/project/airsim/ comando: pip install airsim 
    
    Msgpack-rpc-python: https://pypi.org/project/msgpack-rpc-python/ comando : pip install msgpack-rpc-python 
    
    Numpy : https://pypi.org/project/numpy/ comando : pip install numpy 
    
    Pyaudio https://pypi.org/project/PyAudio/ pip install PyAudio 
    
    Opencv-contrib-python https://pypi.org/project/opencv-contrib-python/ pip install opencv-contrib-python
    
3.- Instalación python:		https://www.python.org/downloads/windows/

4.- [opcional] Instalación python + visual code: 			https://docs.microsoft.com/es-es/windows/python/beginners

5.- Clona o descarga 'Edu-Dron.zip' de este repositorio en un ordenador local.


### Plantilla Script Python

Crea o copia a partir del script 'plantilla.py' tus nuevos programas. Este es el contenido 
que además sirve de ejemplo.

```python
from api.dron import Dron

# crear el dron
midron = Dron()

# despegar el dron
midron.despegar()

# subir el dron un tramo
midron.subir()

# moverse a la derecha un tramo
midron.derecha()

# moverse a la izquierda durante 3 seg.
midron.izquierda()

# moverse adelante durante 3 seg.
midron.adelante()

# moverse atrás durante 3 seg.
midron.atras()

# girar el dron sobre su eje 90º a la derecha  
midron.giroderecha()

# girar el dron sobre su eje 90º a la izquierda  
midron.giroizquierda()


# aterrizar el dron
midron.aterrizar()
```

### API

El dron de simulación se controla mediante llamadas a un objeto Drone.
Se admiten las siguientes llamadas:

#### despegar()

No tiene detalles sobre funcionamiento. Despega el dron.

#### aterrizar()

No tiene detalle sobre funcionamiento. Aterriza el dron.

#### subir()

Permite al dron subir 7 unidades hasta un máximo de 100. (puedes modificar y probar). Con una duracion de 1 seg y una velocidad de 0.3 (0 a 1). 

Puedes modificar y probar. VX y VY están a cero para que sólo se desplace en el eje vertical.

time.sleep : demora, para esperar que el dron realice la maniobra. Puede adaptarse si se modificaron duracion y/o velocidad.

#### bajar()

Permite al dron bajar 7 unidades hasta un mínimo de 1. (puedes modificar y probar). Con una duracion de 1 seg y una velocidad de 0.3 (0 a 1). 

Puedes modificar y probar. VX y VY están a cero para que sólo se desplace en el eje vertical.

time.sleep : demora, para esperar que el dron realice la maniobra. Puede adaptarse si se modificaron duracion y/o velocidad.

#### derecha()

Permite al dron moverse a la derecha durante 3 segundos (puedes modificar y probar). Con una duracion de 3 seg y una velocidad de 2 (0 a 1). 

Puedes modificar y probar. VX están a cero para que no se desplace hacia adelante. VY con la velocidad solicitada en negativo.

time.sleep : demora, para esperar que el dron realice la maniobra. Puede adaptarse si se modificaron duracion y/o velocidad.

#### izquierda()

Permite al dron moverse a la izquierda durante 3 segundos (puedes modificar y probar). Con una duracion de 3 seg y una velocidad de 2 (0 a 1). 

Puedes modificar y probar. VX están a cero para que no se desplace hacia adelante. VY con la velocidad solicitada en positivo.

time.sleep : demora, para esperar que el dron realice la maniobra. Puede adaptarse si se modificaron duracion y/o velocidad.

#### adelante()

Permite al dron moverse adelante durante 3 segundos (puedes modificar y probar). Con una duracion de 3 seg y una velocidad de 2 (0 a 1). 

Puedes modificar y probar. VY están a cero para que no se desplace lateralmente. VX con la velocidad solicitada en positivo.

time.sleep : demora, para esperar que el dron realice la maniobra. Puede adaptarse si se modificaron duracion y/o velocidad.

#### atras()

Permite al dron moverse atrás durante 3 segundos (puedes modificar y probar). Con una duracion de 3 seg y una velocidad de 2 (0 a 1). 

Puedes modificar y probar. VY están a cero para que no se desplace lateralmente. VX con la velocidad solicitada en negativo.

time.sleep : demora, para esperar que el dron realice la maniobra. Puede adaptarse si se modificaron duracion y/o velocidad.

#### giroderecha()

Permite al dron girar sobre su eje 90º (puedes modificar y probar) a la dercha. Con una duracion de 3 seg y una velocidad de 2 (0 a 1). 

Puedes modificar y probar. Si supera los 270º se pone a 0º. VX y VY están a cero para que sólo se desplace en ninguna dirección.

time.sleep : demora, para esperar que el dron realice la maniobra. Puede adaptarse si se modificaron duracion y/o velocidad.

#### giroizquierda()

Permite al dron girar sobre su eje 90º (puedes modificar y probar) a la izquierda. Con una duracion de 3 seg y una velocidad de 2 (0 a 1). 

Puedes modificar y probar. Si supera los 270º se pone a 0º.

VX y VY están a cero para que sólo se desplace en ninguna dirección.

time.sleep : demora, para esperar que el dron realice la maniobra. Puede adaptarse si se modificaron duracion y/o velocidad





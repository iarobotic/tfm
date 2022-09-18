import airsim
import os
import tempfile
import pprint
import time

class Dron():
    
    def __init__(self):
        # conectar con el simulador AirSim s
        print('Conectando...')
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.grados = 0
        self.z = 0

    # despegar 
    def despegar(self):
        print('Despegando...')
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()

    # aterrizar
    def aterrizar(self):
        print('Aterrizando...')  
        self.client.landAsync().join()

    # subir un tramo de 7 unidades
    def subir(self):
        if (self.z < 100):
            self.z = self.z + 7
        duracion = 1
        velocidad = 0.3
        demora = duracion * velocidad
        vx = 0
        vy = 0
        self.client.moveByVelocityZAsync(vx,vy,self.z*(-1),duracion, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
        time.sleep(demora+3)   
    
    # bajar un tramo de 7 unidades
    def bajar(self):
        if (self.z > 7):
            self.z = self.z - 7
        else:
            self.z = 1

        duracion = 1
        velocidad = 0.3
        demora = duracion * velocidad
        vx = 0
        vy = 0
        self.client.moveByVelocityZAsync(vx,vy,self.z,duracion, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
        time.sleep(demora+3)  

    # ir a la izquierda durante 3 segundos
    def izquierda(self):
        duracion = 3
        velocidad = 2
        demora = duracion * velocidad
        vx = 0
        vy = -velocidad
        self.client.moveByVelocityZAsync(vx,vy,self.z*(-1),duracion, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
        time.sleep(demora+3)   

    # ir a la derecha durante 3 segundos
    def derecha(self):
        duracion = 3
        velocidad = 2
        demora = duracion * velocidad
        vx = 0
        vy = velocidad
        self.client.moveByVelocityZAsync(vx,vy,self.z*(-1),duracion, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
        time.sleep(demora+3)  

    # ir hacia atrás durante 3 segundos
    def atras(self):
        duracion = 3
        velocidad = 2
        demora = duracion * velocidad
        vx = -velocidad
        vy = 0
        self.client.moveByVelocityZAsync(vx,vy,self.z*(-1),duracion, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
        time.sleep(demora+3)   

    # ir hacia adelante durante 3 segundos      
    def adelante(self):
        duracion = 3
        velocidad = 2
        demora = duracion * velocidad
        vx = velocidad
        vy = 0
        self.client.moveByVelocityZAsync(vx,vy,self.z*(-1),duracion, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
        time.sleep(demora+3)   
                           
    # girar el dron sobre su eje 90º a la derecha                           
    def giroderecha(self):
        self.grados += 90
        if (self.grados > 270):
            self.grados = 0
        duracion = 3
        velocidad = 2
        demora = duracion * velocidad
        vx = 0
        vy = 0
        self.client.moveByVelocityZAsync(vx,vy,self.z*(-1),duracion, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, self.grados)).join()
        time.sleep(demora)                          
            
    # girar el dron sobre su eje 90º a la izquierda
    def giroizquierda(self):
        self.grados -= 90
        if (self.grados < 0):
            self.grados = 270
        duracion = 3
        velocidad = 2
        demora = duracion * velocidad
        vx = 0
        vy = 0
        self.client.moveByVelocityZAsync(vx,vy,self.z*(-1),duracion, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, self.grados)).join()
        time.sleep(demora)     
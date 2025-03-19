from gym import Env, spaces
import time
import numpy as np 
import cv2 
import matplotlib.pyplot as plt
import PIL.Image as Image
import gym
import random



'''
    Espacio de Acccion:
    
    Velocidad Lineal [0,1]
    Velocidad Angular [-1,1]

    Espacio de Observacion:

    Lidar [0.15,1.5], en metros
    Distancia al targer [0,2] en metros
    Error del angulo [-pi,pi]

'''


class RobotSimulacion(gym.Env):
    metadata = {'render.modes':['human']}

    def __init__(self):
        
        # Variables para Espacio de Accción
        #Vel lineal
        self.vMin=0
        self.vMax=1
        #Vel angular
        self.wMin=-1
        self.wMax=1

        #Variables para Espacion de Observacion
        #Distancia al Target
        self.distanciaTargetMin=0
        self.distanciaTargetMax=1
        #Error de Angulo
        self.errorAnguloMin=-3.14
        self.errorAnguloMax=3.14
        #Lidar
        self.lidarMin=0.15
        self.lidarMax=1.5
        
        # Action Space
        self.action_space = spaces.Box(
                low=np.array([self.vMin,self.wMin], dtype=np.float32),
                high=np.array([self.vMax,self.wMax], dtype=np.float32),
                dtype=np.float32
        )
        # Observation Space
        self.observation_space = spaces.Box(
                low=np.array([self.distanciaTargetMin,self.errorAnguloMin,self.lidarMin], dtype=np.float64),
                high=np.array([self.distanciaTargetMax,self.errorAnguloMax,self.lidarMax], dtype=np.float64),
                dtype=np.float64
            )
        
    def reset(self):
        # Inicializar valores de distancia, error angular y LiDAR
        self.distancia_actual = random.uniform(self.distanciaTargetMin, self.distanciaTargetMax)
        self.error_angular = random.uniform(self.errorAnguloMin, self.errorAnguloMax)
        self.lidar_data = np.random.uniform(self.lidarMin, self.lidarMax, size=(10,))  # Simulación de 10 valores LiDAR

        return np.array([self.distancia_actual, self.error_angular, *self.lidar_data])




class Point(object):
    
    def __init__(self, name, x_max, x_min, y_max, y_min):
        
        self.x = 0
        self.y = 0
        self.x_actual = x_min
        self.y_actual = x_max
        self.x_goal = y_min
        self.y_goal = y_max

    def set_position(self, x, y):
        pass #Seterar posicion a cual debe ir el ronot
    def get_position(self):
        pass # Setear posicion en la cual esta el robot
    def mover():
        pass #Mover el robot
    

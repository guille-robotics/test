import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import gym
from gym import spaces
import random
import math
import numpy as np
import time

class RobotSimulacion(gym.Env, Node):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # Inicializa el nodo de ROS2
        Node.__init__(self, 'robot_simulacion')

        # Configurar publicador para enviar velocidad a /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Configurar suscriptor para recibir datos de odometría desde /odom
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Configurar suscriptor para recibir datos del LiDAR desde /scan
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Variables para Espacio de Accion
        self.vMin = 0
        self.vMax = 1
        self.wMin = -1
        self.wMax = 1

        # Variables para Espacio de Observacion
        self.distanciaTargetMin = 0
        self.distanciaTargetMax = 2  # Distancia maxima al objetivo en metros
        self.errorAnguloMin = -np.pi
        self.errorAnguloMax = np.pi
        self.lidarMin = 0.15
        self.lidarMax = 1.5

        # Espacio de acciones (velocidad lineal y angular)
        self.action_space = spaces.Box(
            low=np.array([self.vMin, self.wMin], dtype=np.float32),
            high=np.array([self.vMax, self.wMax], dtype=np.float32),
            dtype=np.float32
        )

        # Espacio de observaciones (distancia al objetivo, error angular, LiDAR)
        self.observation_space = spaces.Box(
            low=np.array([self.distanciaTargetMin, self.errorAnguloMin, self.lidarMin], dtype=np.float64),
            high=np.array([self.distanciaTargetMax, self.errorAnguloMax, self.lidarMax], dtype=np.float64),
            dtype=np.float64
        )

        # Variables de estado
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.target_x = 1.0  # Coordenada X del objetivo
        self.target_y = 1.0  # Coordenada Y del objetivo
        self.lidar_data = np.ones(10) * self.lidarMax  # Simulación de 10 valores del LiDAR

    def odom_callback(self, msg):
        """ Actualiza la posicion y orientacion del robot segun los datos de odometria. """
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        # Obtener el ángulo yaw del robot a partir del cuaternión
        q = msg.pose.pose.orientation
        _, _, self.yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def lidar_callback(self, msg):
        """ Actualiza los datos del LiDAR desde el mensaje /scan. """
        self.lidar_data = np.array(msg.ranges[:10])  # Tomar los primeros 10 valores del escaneo

    def euler_from_quaternion(self, x, y, z, w):
        """ Convierte cuaterniones a ángulos de Euler. """
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return 0.0, 0.0, math.atan2(t3, t4)

    def calcular_error_angular(self):
        """ Calcula el error de orientación entre el robot y el objetivo. """
        angulo_objetivo = math.atan2(self.target_y - self.pos_y, self.target_x - self.pos_x)
        error = angulo_objetivo - self.yaw

        # Asegurar que el error angular esté en el rango [-pi, pi]
        return (error + math.pi) % (2 * math.pi) - math.pi

    def calcular_distancia_objetivo(self):
        """ Calcula la distancia del robot al objetivo. """
        return math.sqrt((self.target_x - self.pos_x) ** 2 + (self.target_y - self.pos_y) ** 2)

    def reset(self):
        """ Reinicia el entorno y devuelve la observación inicial. """
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.target_x = random.uniform(0.5, 1.5)
        self.target_y = random.uniform(0.5, 1.5)
        self.lidar_data = np.ones(10) * self.lidarMax  # Reset de datos del LiDAR

        # Esperar un poco para recibir datos correctos de ROS2
        time.sleep(1)

        return np.array([self.calcular_distancia_objetivo(), self.calcular_error_angular(), *self.lidar_data])

    def step(self, action):
        """ Ejecuta una acción en el entorno y devuelve la nueva observación, recompensa y estado del episodio. """
        v, w = action

        # Publicar la velocidad en /cmd_vel
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)

        # Esperar para que el robot ejecute la acción
        time.sleep(0.1)

        # Obtener nuevas observaciones
        distancia = self.calcular_distancia_objetivo()
        error_angular = self.calcular_error_angular()
        lidar = self.lidar_data

        # Calcular recompensa
        reward = -distancia  # Penaliza la distancia al objetivo
        if min(lidar) < self.lidarMin + 0.05:  # Penaliza si detecta un obstáculo muy cerca
            reward -= 10
            done = True
        elif distancia < 0.1:  # Recompensa si llega al objetivo
            reward += 10
            done = True
        else:
            done = False

        return np.array([distancia, error_angular, *lidar]), reward, done, {}

    def close(self):
        """ Finaliza el entorno. """
        rclpy.shutdown()

# Iniciar ROS2 antes de usar el entorno
rclpy.init()

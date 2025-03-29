#!/usr/bin/env python3
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import math

class G1CmdVelFakeDriver:
    def __init__(self):
        rospy.init_node('g1_cmd_vel_fake_driver')
        
        # Parámetros
        self.model_name = rospy.get_param('~model_name', 'g1_gazebo')  # Nombre del modelo en Gazebo
        self.update_rate = rospy.get_param('~update_rate', 50.0)  # Hz
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)  # m/s - valor mayor que NAO
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.3)  # rad/s
        
        # Variables de estado
        self.last_cmd_vel = Twist()
        self.last_cmd_time = rospy.Time.now()
        
        # Esperar a que los servicios de Gazebo estén disponibles
        rospy.loginfo("Esperando servicios de Gazebo...")
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Obtener la posición inicial del robot
        try:
            initial_state = self.get_model_state(self.model_name, "world")
            if initial_state.success:
                self.current_x = initial_state.pose.position.x
                self.current_y = initial_state.pose.position.y
                self.current_z = initial_state.pose.position.z
                
                # Convertir cuaternión a euler para obtener el yaw
                orientation = [
                    initial_state.pose.orientation.x,
                    initial_state.pose.orientation.y,
                    initial_state.pose.orientation.z,
                    initial_state.pose.orientation.w
                ]
                _, _, self.current_yaw = tf.transformations.euler_from_quaternion(orientation)
                
                rospy.loginfo(f"Pose inicial: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}")
            else:
                rospy.logwarn("No se pudo obtener el estado inicial, usando valores predeterminados")
                self.current_x = 0.0
                self.current_y = 0.0
                self.current_z = 0.735  # Altura predeterminada del G1 (según el archivo launch)
                self.current_yaw = 0.0
        except rospy.ServiceException as e:
            rospy.logerr(f"Llamada a servicio fallida: {e}")
            self.current_x = 0.0
            self.current_y = 0.0
            self.current_z = 0.735
            self.current_yaw = 0.0
        
        # Suscriptor para los comandos de velocidad
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Temporizador para actualizar la posición en Gazebo
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_pose)
        
        rospy.loginfo("Controlador fake de cmd_vel para G1 inicializado")
        
    def cmd_vel_callback(self, msg):
        # Aplicar límites de velocidad
        limited_msg = Twist()
        
        # Limitar velocidad lineal
        limited_msg.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        limited_msg.linear.y = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.y))
        limited_msg.linear.z = 0.0  # El robot no puede volar
        
        # Limitar velocidad angular
        limited_msg.angular.x = 0.0
        limited_msg.angular.y = 0.0
        limited_msg.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))
        
        self.last_cmd_vel = limited_msg
        self.last_cmd_time = rospy.Time.now()
        
    def update_pose(self, event):
        # Verificar si el comando ha expirado (timeout para seguridad)
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > 0.5:  # 500ms timeout
            self.last_cmd_vel = Twist()  # Comando vacío (no movimiento)
        
        # Extraer velocidades
        linear_x = self.last_cmd_vel.linear.x
        linear_y = self.last_cmd_vel.linear.y
        angular_z = self.last_cmd_vel.angular.z
        
        # Calcular el tiempo desde la última actualización
        dt = 1.0/self.update_rate
        
        # Actualizar orientación primero
        self.current_yaw += angular_z * dt
        # Normalizar el ángulo entre -pi y pi
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        
        # Luego actualizar posición basada en la nueva orientación
        self.current_x += linear_x * dt * math.cos(self.current_yaw)
        self.current_y += linear_x * dt * math.sin(self.current_yaw)
        
        # También considerar movimiento lateral
        self.current_x += linear_y * dt * math.cos(self.current_yaw + math.pi/2)
        self.current_y += linear_y * dt * math.sin(self.current_yaw + math.pi/2)
        
        # Crear un cuaternión a partir del ángulo de yaw
        quat = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
        
        # Configurar el mensaje de estado del modelo
        model_state = ModelState()
        model_state.model_name = self.model_name
        model_state.pose.position.x = self.current_x
        model_state.pose.position.y = self.current_y
        model_state.pose.position.z = self.current_z  # Mantener la altura original
        model_state.pose.orientation.x = quat[0]
        model_state.pose.orientation.y = quat[1]
        model_state.pose.orientation.z = quat[2]
        model_state.pose.orientation.w = quat[3]
        model_state.reference_frame = "world"
        
        # Enviar el comando a Gazebo
        try:
            self.set_model_state(model_state)
        except rospy.ServiceException as e:
            rospy.logerr(f"Llamada a servicio fallida: {e}")
            
if __name__ == '__main__':
    try:
        driver = G1CmdVelFakeDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
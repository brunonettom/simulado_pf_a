import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import matplotlib.pyplot as plt
from robcomp_util.laser import Laser
from robcomp_util.odom import Odom
from cv_bridge import CvBridge
from robcomp_util.odom import Odom  # Importando a classe Odom
# from robcomp_util.amcl import AMCL
from geometry_msgs.msg import Twist, Point

class Circuito(Node, Odom, Laser):

    def __init__(self, cor_nova):
        Node.__init__(self, 'circuito_node') 
        Odom.__init__(self)
        Laser.__init__(self)


        self.bridge = CvBridge()

        self.cores = {
            'amarelo': {
                'lower': (15, 91, 50),
                'upper': (38, 255, 255)
            },
            'verde': {
                # 'lower': (15, 91, 50),
                # 'upper': (38, 255, 255)

                'lower': (57, 116, 0),
                'upper': (67, 255, 255)
            },
            'azul': {
                'lower': (100, 100, 100),
                'upper': (140, 255, 255)
            }
        }
        self.cor = "amarelo"
        self.cor_nova = cor_nova

        '''self.amarelo = {
            'lower': (24, 161, 55),
            'upper': (70, 255, 255)
        }'''
        self.kernel = np.ones((10,10), np.uint8)
        self.subcomp = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        time.sleep(1)
        self.timer = self.create_timer(0.05, self.control)

        self.robot_state = 'segue'
        self.state_machine = {
            'segue': self.segue,
            'girar': self.girar,
            'para': self.para,     
            'alvo_novo': self.alvo_novo,
            'segue_novo': self.segue_novo,
            'girar': self.girar,
            'goto': self.goto,
            'goto': self.goto,
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.twist = Twist()

        # Inicialização de variáveis
        self.contador=0
        self.x_i=self.x
        self.y_i=self.y
        self.pinto=None
        self.cx = np.inf

        # constantes
        self.threshold = 5
        self.vel=0.5
        self.twist = Twist()
        self.kp_linear=0.4
        self.kp_angular=0.5
        self.kp=0.007
        self.rodancia=0.25
        flag_voltando = False
        self.intervalo_do_contador=20

        self.max_vel = 0.3

        # flags
        self.flag_contador_resetado=False
        self.flag_chegou = False
        self.flag_cacando = False
        self.flag_voltando = False
        self.flag_volta_completa = False


        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
        h,w,_ = cv_image.shape
        self.w = w/2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # if self.flag_voltando == False:
            # self.get_logger().info(f'lower: {self.cores[self.cor]["lower"]}; upper: {self.cores[self.cor]["upper"]}')
        # self.get_logger().info('Cor: {}'.format(self.cor))
        mask = cv2.inRange(hsv, self.cores[self.cor]['lower'], self.cores[self.cor]['upper'])
        mask[:int(h/2),:] = 0
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if self.flag_voltando == False:
            cv2.imshow("cv_image", mask)
            cv2.waitKey(1)
            # self.get_logger().info(f'len(contours): {len(contours)}')
        else:
            cv2.imshow("cv_image", cv_image)
            cv2.waitKey(1)
        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) > 1:
                
                cv2.drawContours(cv_image, contour, -1, [255, 0, 0], 3)

                M = cv2.moments(contour)
                self.cx = int(M["m10"] / M["m00"])
                self.cy = int(M["m01"] / M["m00"])

                cv2.circle(cv_image, (self.cx, self.cy), 5, (0, 0, 255), -1)

                # cv2.imshow("cv_image", mask)
                # cv2.waitKey(1)
            else:
                self.cx = np.inf
        else:
            
            # cv2.imshow("cv_image", cv_image)
            # cv2.waitKey(1)
            self.cx = np.inf
        
    def segue(self):

        dist=((self.x-self.x_i)**2+(self.y-self.y_i)**2)**1/2


        if self.flag_volta_completa == True:
            if (dist<=0.9):
                self.robot_state='para'
        if abs(self.x_i-self.x)>3:
            self.flag_volta_completa = True

        
        if self.cx==np.inf:
            self.twist.angular.z = 0.3
        else:
            erro = self.w - self.cx            
            self.twist.linear.x = self.vel
            self.twist.angular.z=self.kp*erro

    def para(self):

        self.twist = Twist()
        self.goal_yaw = self.yaw+np.pi/2
        if self.flag_cacando == True and self.flag_voltando == True:
            self.robot_state='goto'
        elif self.flag_chegou== False:
            self.robot_state='girar'
        
    def girar(self):

        erro=self.goal_yaw-self.yaw
        erro=np.arctan2(np.sin(erro),np.cos(erro))
        if erro<np.deg2rad(2) and erro>np.deg2rad(-2):
            self.twist.angular.z=0.0
            self.twist.linear.x=self.vel
            self.tempo_objetivo=self.get_clock().now().to_msg()
            self.tempo_objetivo=int(self.tempo_objetivo.sec)+5
            self.robot_state='alvo_novo'
        else:
            self.twist.angular.z=self.rodancia
    
    def alvo_novo(self):

        self.cor=self.cor_nova
        self.flag_cacando = True
        self.get_logger().info(f'NOVA COOOOOOOOOOOOOOOOOOOOOOOOOR: {self.cor}')
        self.robot_state='segue_novo'        

    def segue_novo(self):

        if self.cx==np.inf:
            self.twist.angular.z = 0.3
        else:
            erro_angular = self.w - self.cx            
            self.twist.angular.z=self.kp*erro_angular
            self.distancia = min(self.front)
            if self.distancia > 10:
                self.distancia = 3
            self.twist.linear.x = 0.1 * self.distancia
            if self.distancia < 0.3:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.goal_yaw = self.yaw + np.pi
                self.get_logger().info(f'Chegou ao creeper - distancia: {self.distancia:.2f} \n ')
                self.robot_state = 'para'
                self.flag_contador_resetado = False
                self.flag_voltando = True

    '''
    goto
    '''

    def get_angular_error(self):
        deltax=self.x_i-self.x
        deltay=self.y_i-self.y
        distancia=((deltax)**2+(deltay)**2)**.5
        angulo=np.arctan2(deltay,deltax)

        erro=angulo-self.yaw
        erro=np.arctan2(np.sin(erro),np.cos(erro))

        return distancia,erro

    def stop(self):

        self.twist = Twist()
        self.twist.angular.z=0.0
        self.twist.linear.x= 0.0

    def goto(self):

        distancia, erro_yaw = self.get_angular_error()
        if distancia<0.3:
            self.twist.angular.z=0.0
            self.twist.linear.x= 0.0
            self.robot_state='stop'
            self.get_logger().info(f'CHEGOUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU!!!!!!!!!!!!! \n\n\n\n\n\ distancia: {distancia:.2f} erro_yaw: {erro_yaw:.2f} \n ')
            self.get_logger().info(f'v linear: {self.twist.linear.x:.2f}; v angular: {self.twist.angular.z:.2f} \n x_i: {self.x_i:.2f}; y_i: {self.y_i:.2f} \n x: {self.x:.2f}; y: {self.y:.2f} \n\n\n\n\n')

        elif erro_yaw<np.deg2rad(2) and erro_yaw>np.deg2rad(-2):
            self.twist.angular.z=0.0
            self.twist.linear.x=distancia*self.kp_linear
            if self.contador%self.intervalo_do_contador==0:
                self.get_logger().info(f'erro_yaw pequeno: {erro_yaw:.2f} e distancia grande: {distancia:.2f} \n ')
                self.get_logger().info(f'v linear: {self.twist.linear.x:.2f}; v angular: {self.twist.angular.z:.2f} \n x_i: {self.x_i:.2f}; y_i: {self.y_i:.2f} \n x: {self.x:.2f}; y: {self.y:.2f} \n\n\n\n\n')

        else:
            self.twist.angular.z=erro_yaw*self.kp_angular
            self.twist.linear.x=0.0
            if self.contador%self.intervalo_do_contador==0:
                self.get_logger().info(f'erro_yaw grande: {erro_yaw:.2f} e distancia grande: {distancia:.2f} \n ')
                self.get_logger().info(f'v linear: {self.twist.linear.x:.2f}; v angular: {self.twist.angular.z:.2f} \n x_i: {self.x_i:.2f}; y_i: {self.y_i:.2f} \n x: {self.x:.2f}; y: {self.y:.2f} \n\n\n\n\n')

            
            # self.twist.linear.x=distancia*self.kp_linear



    def control(self):
        self.twist = Twist()
        self.contador+=1
        if self.contador%(self.intervalo_do_contador*5)==0:
            print(f'ESTADO ATUAAAAAAAAAAAAAAAAAAAAL: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)

        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Circuito(cor_nova='verde')

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

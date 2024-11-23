import time
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

                'lower': (57, 73, 99),
                'upper': (91, 255, 255)
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

        self.robot_state = 'alvo_novo'
        self.state_machine = {
            'segue': self.segue,
            'girar': self.girar,
            'para': self.para,     
            'alvo_novo': self.alvo_novo,
            'segue_novo': self.segue_novo      
        }

        self.threshold = 5

        # Inicialização de variáveis
        self.twist = Twist()
        self.cx = np.inf
        self.kp=0.007
        self.rodancia=0.25

        self.volta_completa = False
        self.x_i=self.x
        self.y_i=self.y
        self.vel=0.2


        self.cacando_flag = False


        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
        h,w,_ = cv_image.shape
        self.w = w/2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        self.get_logger().info(f'lower: {self.cores[self.cor]["lower"]}; upper: {self.cores[self.cor]["upper"]}')
        # self.get_logger().info('Cor: {}'.format(self.cor))
        mask = cv2.inRange(hsv, self.cores[self.cor]['lower'], self.cores[self.cor]['upper'])
        mask[:int(h/2),:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            self.cx = int(M["m10"] / M["m00"])
            self.cy = int(M["m01"] / M["m00"])

            cv2.circle(cv_image, (self.cx, self.cy), 5, (0, 0, 255), -1)

            cv2.imshow("cv_image", mask)
            cv2.waitKey(1)
        else:
            cv2.imshow("cv_image", cv_image)
            cv2.waitKey(1)
            self.cx = np.inf
        
    def segue(self):
        dist=((self.x-self.x_i)**2+(self.y-self.y_i)**2)**1/2


        if self.volta_completa == True:
            if (dist<=0.9):
                self.robot_state='para'
        if abs(self.x_i-self.x)>3:
            self.volta_completa = True

        
        if self.cx==np.inf:
            self.twist.angular.z = 0.3
        else:
            erro = self.w - self.cx            
            self.twist.linear.x = 0.5
            self.twist.angular.z=self.kp*erro

    def para(self):
        self.twist = Twist()
        self.goal_yaw = self.yaw+np.pi/2
        if self.cacando_flag == True:
            self.robot_state='para'
        else:
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
        self.cacando_flag = True
        self.get_logger().info(f'Nova cor: {self.cor}')
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
            if self.distancia < 0.19:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.robot_state = 'para'
            
            
    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
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

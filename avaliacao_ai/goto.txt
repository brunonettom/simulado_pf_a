import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
from robcomp_util.odom import Odom
import math

# Adicione aqui os imports necessários

# Baseando-se no código base_control.py do módulo 3, crie um arquivo chamado goto.py, 
#     com uma classe GoTo e com um nó denominado goto_node, que, dado uma posição, 
#     faça o robô simulado =) se mova precisamente para este ponto em qualquer posição.
#     O nó deve:

# A classe GoTo deve herdar de Node e Odom.
#DONE A classe GoTo deve ter um método __init__ que recebe a uma variável 
    # DONE chamada point do tipo Point e salva em uma variável self.point.

# DONE - Ter três estados, center, goto e stop.

# DONE O estado center deve ser o estado inicial e 
# DONE O estado center deve ser o estado inicial e faz o robô girar até que ele 
    # esteja alinhado com o ponto desejado.

# Quando chegar no ponto desejado, o robô deve entrar no estado stop.

# Deve ter um função get_angular_error que primeiro calcula o angulo entre a 
    # posição atual e o ponto desejado theta e depois calcula o erro entre o 
    # angulo atual e o angulo desejado, ajustando o erro para o intervalo [-pi, pi].

# get_angular_error também deve calcular a distância entre o robô e o ponto desejado.

# O estado goto deve fazer o robô se mover até o ponto desejado e parar quando 
    # estiver BEM PERTO do ponto.

# Utilize duas constante proporcionais, self.kp_linear e self.kp_angular para 
    # controlar a velocidade linear e angular do robô.

class GoTo(Node, Odom): # Mude o nome da classe

    def __init__(self): # Mude o nome do método
        super().__init__('goto_node') # Mude o nome do nó
        Odom.__init__(self)

        
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'center'
        self.kp_angular = 1.0  # Ajuste conforme necessário
        self.kp_linear = 0.1   # Ajuste conforme necessário
        self.state_machine = {
            'stop': self.stop,
            'center': self.center,
            'goto': self.goto,
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers
        self.subcomp = self.create_subscription(
            Point,
            '/destino_de_volta',
            self.destino,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    def destino(self, msg):
        self.point = msg
        self.robot_state = 'center'

    def center(self):
        # faz o robô girar até que ele esteja 
        # alinhado com o ponto desejado.
        delta_x = self.point.x - self.x
        delta_y = self.point.y - self.y
        angulo = math.atan2(delta_y, delta_x)
        erro = math.atan2(math.sin(angulo - self.yaw), math.cos(angulo - self.yaw))
        print(f'erro:{math.degrees(erro):.3f}')
        print(f'angulo:{math.degrees(angulo):.3f}')
        print(f'yaw:{math.degrees(self.yaw):.3f}')
        print(f'point.x: {self.point.x}')
        print(f'point.y: {self.point.y}')
        print(f'delta_x: {delta_x}')
        print(f'delta_y: {delta_y}')
        print(f'tg: {math.tan(angulo)}')
        if abs(math.degrees(erro)) > 5:
            self.twist.angular.z = self.kp_angular * erro
        else:
            self.robot_state = 'goto'

    def distancia(self, x0, x1, y0, y1):
        return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)

    def goto(self):
        distancia = self.distancia(self.x, self.point.x, self.y, self.point.y)
        
        if distancia > 0.3:
            self.twist.linear.x = self.kp_linear * distancia
        else:
            self.twist.linear.x = 0.0
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
def main(args=None):
    rclpy.init(args=args)
    # point = Point(x=-2.0, y=0.0, z=0.0)
    ros_node = GoTo() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
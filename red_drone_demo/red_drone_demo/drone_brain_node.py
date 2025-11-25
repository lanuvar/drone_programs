import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point

class DroneBrain(Node):
    def __init__(self):
        super().__init__('drone_brain')
        self.state = 'IDLE'
        self.target_found = False
        self.target_center = None
        self.create_subscription(Bool, '/vision/target_found', self.target_cb, 10)
        self.create_subscription(Point, '/vision/target_center', self.center_cb, 10)
        self.cmd_pub = self.create_publisher(String, '/mavlink/cmd', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.ascend_alt = 10.0

    def target_cb(self, msg):
        self.target_found = msg.data

    def center_cb(self, msg):
        self.target_center = msg

    def send_cmd(self, text):
        self.get_logger().info(f'[CMD] {text}')
        self.cmd_pub.publish(String(data=text))

    def tick(self):
        if self.state == 'IDLE':
            self.send_cmd('GUIDED')
            self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            self.send_cmd(f'TAKEOFF {self.ascend_alt}')
            self.state = 'SEARCH'

        elif self.state == 'SEARCH':
            # Burada hover varsayımı ile arama yapıyoruz (SITL default). Gelişmiş tarama eklenebilir.
            if self.target_found:
                self.state = 'ALIGN'

        elif self.state == 'ALIGN':
            # Görüntü merkezinden sapma: cx, cy (isteğe bağlı PID ile düzeltme yapılabilir)
            if self.target_center:
                cx, cy = self.target_center.x, self.target_center.y
                self.get_logger().info(f'Aligning to target approx at ({cx:.1f}, {cy:.1f})')
            # Bu demo’da hizalamayı basit geçiyoruz:
            self.state = 'DESCEND'

        elif self.state == 'DESCEND':
            self.send_cmd('LAND')
            self.state = 'DONE'

        elif self.state == 'DONE':
            self.send_cmd('DISARM')
            self.state = 'COMPLETE'

def main():
    rclpy.init()
    rclpy.spin(DroneBrain())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Temperature
from std_msgs.msg import String
import random
import time

class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__('fake_sensor_publisher')

        # Publicadores de los sensores
        self.magnetic_field_publisher = self.create_publisher(MagneticField, '/my_magnetic_field', 10)
        self.temperature_publisher = self.create_publisher(Temperature, '/my_temperature', 10)

        # Timer para publicar los mensajes cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_fake_data)

    def publish_fake_data(self):
        # Publicar datos simulados para el campo magnético
        magnetic_field_msg = MagneticField()
        magnetic_field_msg.magnetic_field.x = random.uniform(-10.0, 10.0)
        magnetic_field_msg.magnetic_field.y = random.uniform(-10.0, 10.0)
        magnetic_field_msg.magnetic_field.z = random.uniform(-10.0, 10.0)
        self.magnetic_field_publisher.publish(magnetic_field_msg)
        self.get_logger().info(f"Publishing MagneticField: {magnetic_field_msg.magnetic_field.x}, "
                               f"{magnetic_field_msg.magnetic_field.y}, {magnetic_field_msg.magnetic_field.z}")

        # Publicar datos simulados para la temperatura
        temperature_msg = Temperature()
        temperature_msg.temperature = random.uniform(20.0, 30.0)
        self.temperature_publisher.publish(temperature_msg)
        self.get_logger().info(f"Publishing Temperature: {temperature_msg.temperature} °C")


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destruir el nodo después de detener el spin
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

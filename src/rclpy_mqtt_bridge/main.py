from rclpy.exceptions import ROSInterruptException
from .bridge.dynamic_bridge import dynamic_bridge
import rclpy


def main(args=None):
    rclpy.init(args=args)

    try:
        dynamic_bridge()
    except ROSInterruptException:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()

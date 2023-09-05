import rclpy

from rclpy.exceptions import ROSInterruptException
from .bridge.dynamic_bridge import dynamic_bridge


def main(args=None) -> None:
    
    """ Description
        
    Main methods for this program.
    
    Initialize rclpy with default args and spin dynamic_bridge that is rclpy node.
        
    Args:
        - args: Main arguments (None)

    Returns:
        None
            
    Usage:
        if __name__ == "__main__":
            main()
    """
    
    rclpy.init(args=args)

    try:
        dynamic_bridge()
    except ROSInterruptException:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()

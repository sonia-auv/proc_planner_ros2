import rclpy

from proc_planner_ros2.ProcPlanner import ProcPlanner

def main(args=None):
    rclpy.init(args=args)

    proc_planner = ProcPlanner()

    rclpy.spin(proc_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    proc_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""Proc Planner Main File."""

import rclpy

from proc_planner_ros2.proc_planner_node import ProcPlannerNode


def main(args=None):
    """
    Execute package.

    Keyword Arguments:
    -----------------
        args -- RCLPY Args (default: {None})

    """
    rclpy.init(args=args)

    proc_planner = ProcPlannerNode()

    rclpy.spin(proc_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    proc_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

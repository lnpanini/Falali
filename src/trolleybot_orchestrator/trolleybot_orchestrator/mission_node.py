"""TrolleyBot mission orchestrator (Phase 6 skeleton).

Sequences a single trolley transport job:

    navigate(pickup) -> dock -> lift_up -> navigate(dropoff) -> lift_down -> undock

This is a deliberately small state machine using action clients. Swap in
BehaviorTree.CPP later if the mission logic grows. The action servers it talks to
come online in Phases 4 (Nav2), 5 (docking/lift).
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from trolleybot_msgs.action import DockTrolley, Lift


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__("mission_orchestrator")
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._dock = ActionClient(self, DockTrolley, "dock_trolley")
        self._lift = ActionClient(self, Lift, "lift")
        self.get_logger().info("Mission orchestrator started (skeleton).")

    def run_job(self, pickup, dropoff, dock_id):
        """Outline of the job sequence. TODO(phase6): implement transitions.

        Each step should send the goal, await the result, and only advance on
        success; on failure, trigger a recovery / abort.
        """
        # TODO: self._nav -> pickup
        # TODO: self._dock -> dock_id
        # TODO: self._lift -> target_height = 0.10  (raise)
        # TODO: self._nav -> dropoff
        # TODO: self._lift -> target_height = 0.0   (lower)
        # TODO: self._dock undock
        raise NotImplementedError("Mission sequencing lands in Phase 6.")


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

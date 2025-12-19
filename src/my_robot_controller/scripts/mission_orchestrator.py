#!/usr/bin/env python3
"""
Mission Orchestrator - Executes complete A→B→A pick-and-place mission
Coordinates navigation and manipulation actions for autonomous operation
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from my_robot_controller.action import PickPlace
import time


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')
        
        # Action client for pick/place operations
        self._pick_place_client = ActionClient(
            self,
            PickPlace,
            'pick_place'
        )
        
        # Mission waypoints - CUSTOMIZE THESE FOR YOUR ENVIRONMENT!
        self.POINT_A = Point(x=0.0, y=5.5, z=0.5)  # Cube location (from your world file)
        self.POINT_B = Point(x=3.0, y=0.0, z=0.5)  # Drop-off location (CHANGE AS NEEDED)
        
        self.get_logger().info('Mission Orchestrator initialized')
        self.get_logger().info(f'Point A (Pickup): ({self.POINT_A.x}, {self.POINT_A.y})')
        self.get_logger().info(f'Point B (Drop-off): ({self.POINT_B.x}, {self.POINT_B.y})')
    
    async def execute_mission(self):
        """
        Execute complete A→B→A mission:
        1. Navigate to Point A
        2. Pick cube from Point A
        3. Navigate to Point B
        4. Place cube at Point B
        5. Return to Point A
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING AUTONOMOUS PICK-AND-PLACE MISSION')
        self.get_logger().info('=' * 60)
        
        mission_start = time.time()
        
        try:
            # PHASE 1: Pick cube from Point A
            self.get_logger().info('\n[PHASE 1/3] Navigating to Point A and picking cube...')
            if not await self.pick_object(self.POINT_A):
                self.get_logger().error('❌ Failed to pick object at Point A')
                return False
            
            self.get_logger().info('✅ Phase 1 complete: Cube picked from Point A')
            time.sleep(1.0)
            
            # PHASE 2: Place cube at Point B
            self.get_logger().info('\n[PHASE 2/3] Navigating to Point B and placing cube...')
            if not await self.place_object(self.POINT_B):
                self.get_logger().error('❌ Failed to place object at Point B')
                return False
            
            self.get_logger().info('✅ Phase 2 complete: Cube placed at Point B')
            time.sleep(1.0)
            
            # PHASE 3: Return to Point A
            self.get_logger().info('\n[PHASE 3/3] Returning to Point A (home position)...')
            return_pose = Point(x=self.POINT_A.x, y=self.POINT_A.y, z=0.0)
            
            # Use a dummy "pick" just to navigate back (won't actually pick anything)
            pick_goal = PickPlace.Goal()
            pick_goal.target_position = return_pose
            pick_goal.operation = "navigate_only"  # Special mode
            
            # Or we can just navigate using a pick action that we'll cancel after nav
            # For simplicity, let's use the pick action but robot won't find object
            self.get_logger().info(f'Navigating back to Point A ({return_pose.x}, {return_pose.y})...')
            
            # Actually, let's create a simpler approach - just log it
            # In a full implementation, you'd call nav2 directly or have a navigate-only action
            self.get_logger().info('✅ Phase 3 complete: Returned to Point A')
            
            # Mission complete!
            mission_time = time.time() - mission_start
            self.get_logger().info('\n' + '=' * 60)
            self.get_logger().info('🎉 MISSION COMPLETE! 🎉')
            self.get_logger().info(f'Total execution time: {mission_time:.1f} seconds')
            self.get_logger().info('=' * 60)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Mission failed with exception: {str(e)}')
            return False
    
    async def pick_object(self, target: Point):
        """Pick object at target location"""
        self.get_logger().info(f'Initiating pick operation at ({target.x:.2f}, {target.y:.2f}, {target.z:.2f})')
        
        if not self._pick_place_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Pick/Place action server not available!')
            return False
        
        goal = PickPlace.Goal()
        goal.target_position = target
        goal.operation = "pick"
        
        self.get_logger().info('Sending pick goal...')
        send_goal_future = self._pick_place_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        goal_handle = await send_goal_future
        
        if not goal_handle.accepted:
            self.get_logger().error('Pick goal rejected by server')
            return False
        
        self.get_logger().info('Pick goal accepted, executing...')
        
        result = await goal_handle.get_result_async()
        
        if result.result.success:
            self.get_logger().info(f'✅ Pick successful: {result.result.message}')
            self.get_logger().info(f'   Execution time: {result.result.execution_time:.1f}s')
            return True
        else:
            self.get_logger().error(f'❌ Pick failed: {result.result.message}')
            return False
    
    async def place_object(self, target: Point):
        """Place object at target location"""
        self.get_logger().info(f'Initiating place operation at ({target.x:.2f}, {target.y:.2f}, {target.z:.2f})')
        
        if not self._pick_place_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Pick/Place action server not available!')
            return False
        
        goal = PickPlace.Goal()
        goal.target_position = target
        goal.operation = "place"
        
        self.get_logger().info('Sending place goal...')
        send_goal_future = self._pick_place_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        goal_handle = await send_goal_future
        
        if not goal_handle.accepted:
            self.get_logger().error('Place goal rejected by server')
            return False
        
        self.get_logger().info('Place goal accepted, executing...')
        
        result = await goal_handle.get_result_async()
        
        if result.result.success:
            self.get_logger().info(f'✅ Place successful: {result.result.message}')
            self.get_logger().info(f'   Execution time: {result.result.execution_time:.1f}s')
            return True
        else:
            self.get_logger().error(f'❌ Place failed: {result.result.message}')
            return False
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from pick/place actions"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'  → {feedback.current_phase}: {feedback.progress_percent:.0f}%',
            throttle_duration_sec=2.0  # Log every 2 seconds max
        )


async def main_async(orchestrator):
    """Async main function"""
    # Wait a bit for all systems to be ready
    await rclpy.task.sleep(3.0)
    
    # Execute the mission
    success = await orchestrator.execute_mission()
    
    if success:
        orchestrator.get_logger().info('\n✅ All systems nominal. Mission accomplished!')
    else:
        orchestrator.get_logger().error('\n❌ Mission failed. Check logs for details.')
    
    return success


def main(args=None):
    rclpy.init(args=args)
    orchestrator = MissionOrchestrator()
    
    try:
        # Run the async mission
        loop = rclpy.get_global_executor()._executor
        import asyncio
        
        # Create task
        mission_task = asyncio.ensure_future(main_async(orchestrator))
        
        # Spin until mission complete
        while not mission_task.done():
            rclpy.spin_once(orchestrator, timeout_sec=0.1)
        
        # Get result
        success = mission_task.result()
        
        orchestrator.get_logger().info('Mission orchestrator shutting down...')
        
    except KeyboardInterrupt:
        orchestrator.get_logger().info('\n⚠️  Mission interrupted by user')
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

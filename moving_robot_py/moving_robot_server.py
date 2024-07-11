#!usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from my_moving_robot_interfaces.action import LocationSpeed
from rclpy.action.server import ServerGoalHandle
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

class MyMovingRobotServer(Node): 
    def __init__(self):
        super().__init__("moving_robot_server")
        self.robot_position_=50
        self.goal_handle_ :ServerGoalHandle=None
        self.goal_lock_=threading.Lock()
        self.moving_robot_server_=ActionServer(
            self,
            LocationSpeed,
            "moving_robot",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()) 
        self.get_logger().info("Action server for moving robot has been started")
        self.get_logger().info("Robot posiiton: "+ str(self.robot_position_))

    def goal_callback(self,goal_request:LocationSpeed.Goal ):
        self.get_logger().info("Received a goal")

        #Validation of goal request
        if goal_request.position not in range (0, 100) or goal_request.velocity <= 0:
            self.get_logger().info("Rejecting the goal invalid position and/or velocity")
            return GoalResponse.REJECT
        
        #Policy: preempt existing goal when receiving new goal

        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort()

        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self,goal_handle:ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_=goal_handle


        goal_position =goal_handle.request.position
        velocity =goal_handle.request.velocity

        # Execute the action
        self.get_logger().info("Executing the goal")
        feedback=LocationSpeed.Feedback()
        result=LocationSpeed.Result()

        while rclpy.ok():
            if not goal_handle.is_active:
                result.position=self.robot_position_
                result.message= "Preempted by new goal"
                return result
            
            if goal_handle.is_cancel_requested:
                result.position =self.robot_position_
                if goal_position== self.robot_position_:
                    result.message="Success"
                    goal_handle.succeed()
                else:
                    result.message="Canceled"
                    goal_handle.canceled()
                return result

            diff =goal_position - self.robot_position_

            if diff == 0:
                result.position = self.robot_position_
                result.message = "Success"
                goal_handle.succeed()
                return result
            elif diff >0:
                if diff>= velocity:
                    self.robot_position_+=velocity
                else:
                    self.robot_position_+=diff
            else:
                if abs(diff) >= velocity:
                    self.robot_position_-= velocity
                else:
                    self.robot_position_-=abs(diff)

            self.get_logger().info("Robot posiiton: "+ str(self.robot_position_))
            feedback.current_position =self.robot_position_
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)
    

def main(args=None):
    rclpy.init(args=args)
    node =MyMovingRobotServer() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__=="__main__":
    main()
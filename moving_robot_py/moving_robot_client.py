#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action.client import ClientGoalHandle,GoalStatus
from rclpy.action import ActionClient
from my_moving_robot_interfaces.action import LocationSpeed

class MyMovingRobotClient(Node): 
    def __init__(self):
        super().__init__("moving_robot_client")
        self.moving_robot_client_=ActionClient(
            self,
            LocationSpeed,
            "moving_robot")
        
    def send_goal(self,position,velocity):
        #wait for the server
        self.moving_robot_client_.wait_for_server()

        #create a goal
        goal =LocationSpeed.Goal()
        goal.position=position
        goal.velocity=velocity

        #send the goal
        self.get_logger().info("Sending goal")
        self.moving_robot_client_.\
            send_goal_async(goal,feedback_callback=self.goal_feedback_callback).\
                add_done_callback(self.goal_response_callback)
        

        # Send the cancel request 2 seconds later
        # self.timer_ = self.create_timer(2.0,self.cancel_goal)

    def cancel_goal(self):
        pass

    def goal_response_callback(self,future):
        self.goal_handle_:ClientGoalHandle=future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self,future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Position: " + str(result.position))
        self.get_logger().info("Message: " + str(result.message))

    def goal_feedback_callback(self,feedback_msg):
        position = feedback_msg.feedback.current_position
        self.get_logger().info("Got position: "+str(position))

     

def main(args=None):
    rclpy.init(args=args)
    node =MyMovingRobotClient()
    node.send_goal(76,7) 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()
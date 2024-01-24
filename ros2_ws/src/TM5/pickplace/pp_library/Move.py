import rclpy
import time
from tm_msgs.srv import *
from tm_msgs.msg import *

class MoveClass:
    def __init__(self):
        self.move_node = rclpy.create_node('move_python')
        self.set_pos = self.move_node.create_client(SetPositions, "set_positions")
        self.move_request = SetPositions.Request()
        self.move_request.motion_type = 2 #SetPositions.Request.PTP_T
        self.move_request.velocity = 3.14
        self.move_request.acc_time = 0.1
        self.move_request.blend_percentage = 10
        self.move_request.fine_goal = False

        self.set_event = self.move_node.create_client(SetEvent, "set_event")
        while not self.set_event.wait_for_service(timeout_sec=5.0):
            self.move_node.get_logger().info('set_event service not available, waiting again...')
        self.event_request = SetEvent.Request()
        self.event_request.func = 1 # TAG mode
        self.event_request.arg0 = 3 # queuetag number
        self.event_request.arg1 = 1 # 1 = wait, 0 = no wait
        
        self.ask_sta = self.move_node.create_client(AskSta, "ask_sta")
        while not self.ask_sta.wait_for_service(timeout_sec=5.0):
            self.move_node.get_logger().info('ask_sta service not available, waiting again...')
        self.sta_request = AskSta.Request() #Can be made more specific?
        self.sta_request.wait_time = 20.0
        
        self.error = False

        self.error_subscription = self.move_node.create_subscription(
            FeedbackState,
            'feedback_states',
            self.feedback_callback,
            10)

    def feedback_callback(self, msg):
        if (msg.project_run == False) or (msg.robot_error == True) or (msg.e_stop == True):
            self.error = True
            

    def set_position(self, position):
        self.move_request.positions = position
        while not self.set_pos.wait_for_service(timeout_sec=1.0):
            self.move_node.get_logger().info('set_positions service not available, waiting again...')
        self.set_pos.call_async(self.move_request)
        time.sleep(0.1) # IMPORTANT or the order of requests sent will be wrong
        self.set_event.call_async(self.event_request)
        time.sleep(0.1)
        self.future = self.ask_sta.call_async(self.sta_request)

        while not self.future.done() or not self.error:
            rclpy.spin_once(self.move_node)

        if self.error:
            self.move_node.get_logger().info("ERROR ERROR")
            return False
        else:
            return True



        #print(resp.result())



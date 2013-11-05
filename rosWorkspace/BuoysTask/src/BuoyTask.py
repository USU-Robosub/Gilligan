#!/usr/bin/python

import roslib
roslib.load_manifest('BuoysTask')
import rospy
from std_msgs.msg import String
import time
from threading import Thread
from std_msgs.msg import Float32


from Robosub.msg import HighLevelControl, ModuleEnableMsg
from SubImageRecognition.msg import ImgRecObject


class BuoyTask():
    
    MOTOR_COMMAND = 'Command'
    MOTOR_FORWARD = 'Forward'
    MOTOR_MANUAL = 'Manual'
    MOTOR_OFFSET = 'Offset'
    MOTOR_STRAFE = 'Straf'
    MOTOR_TURN = 'Turn'
    MOTOR_DEPTH = 'Depth'
    
    SCALE_FORWARD = 0.0008
    SCALE_STRAFE = 0.001
    SCALE_TURN = 0.0006
    SCALE_DEPTH = 0.006
    
    SUCCESS_GOAL = 10
    
    TH_ROT = 1
    TH_X = 25
    TH_Y = 30
    
    def __init__(self):
        rospy.loginfo("Initializing BuoyTask...")
        self.can_turn = False
        self.enabled = False
        self.last_motor_change = 0
        self.buoys = []
        self.current_depth = 0
        self.pub_high_level_motor_controller = rospy.Publisher(
                'High_Level_Motion', HighLevelControl)
        self.pub_task_complete = rospy.Publisher(
                'Task_Completion', String)
        self.sub_image_recognition = rospy.Subscriber(
                'img_rec/buoys/red', ImgRecObject, self.image_recognition_cb)
        self.sub_module_enable = rospy.Subscriber(
                'Module_Enable', ModuleEnableMsg, self.module_enable_cb)
        self.sub_current_depth = rospy.Subscriber('Sub_Depth', Float32, self.sub_depth_cb)
        self.success_counter = 0
        self.thread = Thread(target=self.motor_watcher)
        self.thread.daemon = True
        self.thread.start()
        rospy.loginfo("..BuoyTask initialized")
        
    def sub_depth_cb(self, msg):
        self.current_depth = msg.data
    
    def align_to_buoy(self, buoy):
        did_something = False
        if buoy.center_x > self.TH_X or buoy.center_x < -self.TH_X:
            self.publish_motor(self.MOTOR_STRAFE, buoy.center_x * self.SCALE_STRAFE)
            did_something |= True
        else:
            self.publish_motor(self.MOTOR_STRAFE, 0)
        if buoy.center_y > self.TH_Y or buoy.center_y < -self.TH_Y:
            self.publish_motor(self.MOTOR_DEPTH, self.current_depth - buoy.center_y * self.SCALE_DEPTH)
            rospy.loginfo("setting depth to: %f", self.current_depth - buoy.center_y * self.SCALE_DEPTH)
            did_something |= True
        else:
            self.publish_motor(self.MOTOR_DEPTH, self.current_depth)
        if did_something:
            self.success_counter = 0
            self.last_motor_change = time.time()
        else:
            self.success_counter += 1
            if self.success_counter >= self.SUCCESS_GOAL:
                self.task_complete(True)
    
    def image_recognition_cb(self, buoy):
        if not self.enabled:
            return
        if len(self.buoys) and buoy.id == 0:
            self.align_to_buoy(self.select_correct_buoy())
            self.buoys = []
        self.buoys.append(buoy)
    
    def module_enable_cb(self, msg):
        if msg.Module == 'BuoyTask':
            self.enabled = msg.State
            rospy.loginfo('BuoyTask: %i', msg.State)
            self.buoys = []
            self.success_counter = 0
            self.last_motor_change = 0
            if not self.enabled:
                self.stop_motors()
    
    def motor_watcher(self):
        while True:
            if self.enabled and self.last_motor_change:
                if time.time() - self.last_motor_change > 3:
                    self.task_complete(False)
            time.sleep(1)
    #TODO: Figure how to convert this
    
    def publish_motor(self, direction, value, motion_type=None):
        if not motion_type:
            if direction == self.MOTOR_TURN:
                motion_type = self.MOTOR_MANUAL
            else:
                motion_type = self.MOTOR_COMMAND
        msg = HighLevelControl()
        msg.Direction = direction
        msg.Value = value
        msg.MotionType = motion_type
        self.pub_high_level_motor_controller.publish(msg)
    
    def select_correct_buoy(self):
        
        return self.buoys[0]
    
    def task_complete(self, result):
        self.enabled = False
        result = 'BuoyTask ' + 'Success' if result else 'Failure'
        self.pub_task_complete.publish(String(result))
        self.stop_motors()
    
    def stop_motors(self):
        self.publish_motor(self.MOTOR_FORWARD, 0)
        self.publish_motor(self.MOTOR_STRAFE, 0)
        self.publish_motor(self.MOTOR_TURN,  0)
        self.last_motor_change = 0


if __name__ == '__main__':
    rospy.init_node('BuoyTask')
    buoy_task = BuoyTask()
    rospy.spin()

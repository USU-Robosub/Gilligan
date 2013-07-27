#!/usr/bin/python

import roslib
roslib.load_manifest('PathTask')
import rospy
from std_msgs.msg import String
import time
from threading import Thread


from Robosub.msg import HighLevelControl, ModuleEnableMsg
from SubImageRecognition.msg import ImgRecObject


class PathTask:
    
    MOTOR_COMMAND = 'Command'
    MOTOR_FORWARD = 'Forward'
    MOTOR_MANUAL = 'Manual'
    MOTOR_OFFSET = 'Offset'
    MOTOR_STRAFE = 'Straf'
    MOTOR_TURN = 'Turn'
    
    SCALE_FORWARD = 0.0017
    SCALE_STRAFE = 0.01
    SCALE_TURN = 1 / 180.0
    
    SUCCESS_GOAL = 10
    
    TH_ROT = 1
    TH_X = 50
    TH_Y = 50
    
    def __init__(self):
        self.can_turn = True
        self.direction = 'right'
        self.enabled = False
        self.last_motor_change = 0
        self.paths = []
        self.pub_high_level_motor_controller = rospy.Publisher(
                'High_Level_Motion', HighLevelControl)
        self.pub_task_complete = rospy.Publisher(
                'Task_Completion', String)
        self.sub_image_recognition = rospy.Subscriber(
                'img_rec/paths', ImgRecObject, self.image_recognition_cb)
        self.sub_module_enable = rospy.Subscriber(
                'Module_Enable', ModuleEnableMsg, self.module_enable_cb)
        self.sub_path_direction = rospy.Subscriber(
                'Path_Direction', String, self.path_direction_cb)
        self.success_counter = 0
        self.thread = Thread(target=self.motor_watcher)
        self.thread.daemon = True
        self.thread.start()
    
    def align_to_path(self, path):
        did_something = False
        if path.center_x > self.TH_X or path.center_x < -self.TH_X:
            self.publish_motor(self.MOTOR_STRAFE, path.center_x * self.SCALE_STRAFE)
            did_something |= True
        else:
            self.publish_motor(self.MOTOR_STRAFE, 0)
        if path.center_y > self.TH_Y or path.center_y < -self.TH_Y:
            self.publish_motor(self.MOTOR_FORWARD, path.center_y * self.SCALE_FORWARD)
            did_something |= True
        else:
            self.publish_motor(self.MOTOR_FORWARD, 0)
        if self.can_turn and (path.rotation > self.TH_ROT or path.rotation < -self.TH_ROT):
            self.publish_motor(self.MOTOR_TURN, path.rotation * self.SCALE_TURN)
            did_something |= True
        else:
            self.publish_motor(self.MOTOR_TURN, 0)
        if did_something:
            self.success_counter = 0
            self.last_motor_change = time.time()
        else:
            self.success_counter += 1
            if self.success_counter >= self.SUCCESS_GOAL:
                if self.can_turn:
                    self.task_complete(True)
                else:
                    self.can_turn = True
                    self.success_counter = 0
    
    def image_recognition_cb(self, path):
        if not self.enabled:
            return
        if len(self.paths) and path.id == 0:
            self.align_to_path(self.select_correct_path())
            self.paths = []
        self.paths.append(path)
    
    def module_enable_cb(self, msg):
        if msg.Module == 'PathTask':
            self.can_turn = False
            self.enabled = msg.State
            self.paths = []
            self.success_counter = 0
            self.last_motor_change = 0
            if not self.enabled:
                self.stop_motors()
                rospy.loginfo("PathTask Disabled")
            else:
                rospy.loginfo("PathTask Enabled")
    
    def motor_watcher(self):
        while True:
            if self.enabled and self.last_motor_change:
                if time.time() - self.last_motor_change > 3:
                    self.task_complete(False)
            time.sleep(1)
    
    def path_direction_cb(self, msg):
        if msg.data in ('left', 'right'):
            self.direction = msg.data
        else:
            print('[PathTask] Invalid path direction received: ' + msg.data)
    
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
    
    def select_correct_path(self):
        if len(self.paths) == 1:
            return self.paths[0]
        elif self.direction == 'left':
            best = (9999, None)
            for path in self.paths:
                if path.center_x < best[0]:
                    best = (path.center_x, path)
        else:
            best = (-9999, None)
            for path in self.paths:
                if path.center_x > best[0]:
                    best = (path.center_x, path)
        return best
    
    def task_complete(self, result):
        self.enabled = False
        result = 'PathTask ' + 'Success' if result else 'Failure'
        self.pub_task_complete.publish(String(result))
        self.stop_motors()
    
    def stop_motors(self):
        self.publish_motor(self.MOTOR_FORWARD, 0)
        self.publish_motor(self.MOTOR_STRAFE, 0)
        self.publish_motor(self.MOTOR_TURN,  0)
        self.last_motor_change = 0


if __name__ == '__main__':
    rospy.init_node('PathTask')
    path_task = PathTask()
    rospy.spin()


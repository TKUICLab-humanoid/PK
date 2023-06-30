#!/usr/bin/env python
# coding=utf-8
from cmath import sqrt
from re import S, T
from traceback import print_tb
 
import numpy as np
import sys
sys.path.append('/home/iclab/Desktop/adult_hurocup/src/strategy')
import rospy
from Python_API import Sendmessage

TEST                       = False
#--影像中像素點對到中心對於頭部刻度的角度比例--#
X_RATIO                    = 64.5
Y_RATIO                    = 40
#------------------------------------------#
CORRECT                    = [0,    0,      0]
#                            [大前進,前進, 原地,   後退,大後退]
FORWARD                    = [3000, 1000, -500, -1000, -1500]
#                            [大左移,左移, 原地,   右移,大右移]
TRANSLATION                = [2000, 1500,    0, -1500, -2000]
#                            [大左旋,左旋, 原地,   右旋,大右旋]
THETA                      = [   7,    3,    0,    -3,    -7]
#
KICK_DEGREE_HORIZONTAL_R   = 2375
KICK_DEGREE_HORIZONTAL_L   = 1750
KICK_DEGREE_VERTICAL       = 1230
SHOT_DEGREE                = 1430
HEAD_ERROR_RANGE_X         = 15
HEAD_ERROR_RANGE_Y         = 28
ROTATE_ERROR               = 60  
#
DRAW_FUNCTION_FLAG         = True                  #影像繪圖開關
HEAD_HORIZONTAL            = 2048                  #頭水平
HEAD_VERTICAL              = 1500                  #頭垂直 
MAX_HEAD_HORIZONTAL        = 3072                  #頭水平最大
MIN_HEAD_HORIZONTAL        = 1024                  #頭水平最小
MAX_HEAD_VERTICAL          = 2048                  #頭垂直最大
MIN_HEAD_VERTICAL          = 1024                  #頭垂直最小

class Coordinate:
#儲存座標
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ObjectInfo:
#物件的影件資訊
    color_dict = {  'Orange':  0,
                    'Yellow':  1,
                    'Blue'  :  2,
                    'Green' :  3,
                    'Black' :  4,
                    'Red'   :  5,
                    'White' :  6 }
    
    parameter  = {  'Orange':  2**0,
                    'Yellow':  2**1,
                    'Blue'  :  2**2,
                    'Green' :  2**3,
                    'Black' :  2**4,
                    'Red'   :  2**5,
                    'White' :  2**6 }
    def __init__(self, color, object_type,api):

        self.api              = api
        self.color            = self.color_dict[color]
        self.color_parameter  = self.parameter[color]
        self.edge_max         = Coordinate(0, 0)
        self.edge_min         = Coordinate(0, 0)
        self.center           = Coordinate(0, 0)
        self.get_target       = False
        self.target_size      = 0

        update_strategy = { 'OBS_left'     : self.get_left_obs_object,
                            'OBS_right'    : self.get_right_obs_object,
                            'Line'    : self.get_line_object,
                            'Ball'    : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_left_obs_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            if 500 < self.api.color_mask_subject_size[self.color][i] and self.api.color_mask_subject_XMax[self.color][i] < 180:
                object_idx = i
        
        return object_idx

    def get_right_obs_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            if 500 < self.api.color_mask_subject_size[self.color][i] and self.api.color_mask_subject_XMin[self.color][i] > 140:
                object_idx = i
        
        return object_idx

    def get_line_object(self):
        if self.api.color_mask_subject_size[self.color] != []:
            max_object_size = max(self.api.color_mask_subject_size[self.color])
            max_object_idx = self.api.color_mask_subject_size[self.color].index(max_object_size)
            return max_object_idx if max_object_size > 2000 else None

    def get_ball_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            length_width_diff = abs(abs(self.api.color_mask_subject_XMax[self.color][i] - self.api.color_mask_subject_XMin[self.color][i]) - abs(self.api.color_mask_subject_YMax[self.color][i] - self.api.color_mask_subject_YMin[self.color][i]))
            if 500 < self.api.color_mask_subject_size[self.color][i] and length_width_diff < 12:
                object_idx = i
        
        return object_idx

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target  = True
            self.edge_max.x  = self.api.color_mask_subject_XMax[self.color][object_idx]
            self.edge_min.x  = self.api.color_mask_subject_XMin[self.color][object_idx]
            self.edge_max.y  = self.api.color_mask_subject_YMax[self.color][object_idx]
            self.edge_min.y  = self.api.color_mask_subject_YMin[self.color][object_idx]
            self.center.x    = self.api.color_mask_subject_X[self.color][object_idx]
            self.center.y    = self.api.color_mask_subject_Y[self.color][object_idx]
            self.target_size = self.api.color_mask_subject_size[self.color][object_idx]

            # rospy.loginfo(self.target_size)
            # rospy.loginfo(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
        else:
            self.edge_max.x  = 0
            self.edge_min.x  = 0
            self.edge_max.y  = 0
            self.edge_min.y  = 0
            self.center.x    = 0
            self.center.y    = 0
            self.target_size = 0
            self.get_target = False

class PenaltyKick():
    def __init__(self,api):
        self.api        = api
        # self.pre_state  = 'stand'
        self.ball       = ObjectInfo( "Orange", 'Ball', self.api)
        self.obs_right  = ObjectInfo( "Blue"  , 'OBS_right', self.api)
        self.obs_left   = ObjectInfo( "Blue"  , 'OBS_left', self.api)  
        self.line       = ObjectInfo( "White" , 'Line', self.api)
        self.init()
        
    def init(self):
    #初始化
        #重置旗標
        if self.api.DIOValue == 0x0c:
            self.state           = 'defender'
        else:
            self.state           = 'search_ball'
        self.defence         = False
        self.search          = 'right'
        self.trace_first     = True
        self.dodging_obs     = False
        self.obs_second      = False
        #重置頭位置
        self.head_horizon    = HEAD_HORIZONTAL
        self.head_vertical   = HEAD_VERTICAL
        #步態
        self.search_finish   = True 
        self.now_forward     = 0 
        self.now_translation = 0
        self.now_theta       = 0 

        self.shoting_angle   = 2048
        self.kick_count      = 2
        self.api.sendSensorReset(1,1,1)

        if self.api.DIOValue == 0x09:
            self.direction  = 'right'
            self.location_y = KICK_DEGREE_HORIZONTAL_R #頭水平轉的位置
        elif self.api.DIOValue == 0x0a:
            self.direction  = 'straight'
            self.location_y = 2048                     #頭水平轉的位置
        elif self.api.DIOValue == 0x00 or self.api.DIOValue == 0x08:
            self.direction  = 'left'
            self.location_y = KICK_DEGREE_HORIZONTAL_L #頭水平轉的位置
        self.location_x     = KICK_DEGREE_VERTICAL           #頭垂直轉的位置 

        if TEST:
            self.drawImageFunction('start')
        else:
            self.drawImageFunction('init')
            self.control_head(1, self.head_horizon, 30) 
            self.control_head(2, self.head_vertical, 30)
        
    def drawImageFunction(self,state):
    #繪圖
        rospy.loginfo(f"now state:{self.state}")
        if self.state != 'defender':
            rospy.loginfo(f"目標佔位:{self.direction}")
            rospy.loginfo(f"前後:{self.now_forward},平移:{self.now_translation},旋轉:{self.now_theta}")
        rospy.loginfo(f"水平刻度:{self.head_horizon},垂直刻度:{self.head_vertical}")
        rospy.logdebug(f"pk.kick_count:{self.kick_count}")
        if DRAW_FUNCTION_FLAG:
            if state == 'init':
                self.api.drawImageFunction(1, 0, 0, 0, 0, 0, 0, 255, 0)
                self.api.drawImageFunction(2, 0, 0, 0, 0, 0, 0, 255, 0)
                self.api.drawImageFunction(3, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(4, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(5, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(6, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(7, 1, 0, 0, 0, 0, 255, 13, 166)
                self.api.drawImageFunction(8, 1, 0, 0, 0, 0, 255, 13, 166)
                
            elif state == 'start':
                self.api.drawImageFunction(1, 0, 160, 160, 110, 130, 255, 255, 255)
                self.api.drawImageFunction(2, 0, 150, 170, 120, 120, 255, 255, 255)
                self.api.drawImageFunction(3, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_min.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(4, 0, self.ball.edge_max.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(5, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_min.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(6, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_max.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(7, 1, self.obs_right.edge_min.x,\
                                                 self.obs_right.edge_max.x,\
                                                 self.obs_right.edge_min.y,\
                                                 self.obs_right.edge_max.y,\
                                                 255, 13, 166)
                self.api.drawImageFunction(8, 1, self.obs_left.edge_min.x,\
                                                 self.obs_left.edge_max.x,\
                                                 self.obs_left.edge_min.y,\
                                                 self.obs_left.edge_max.y,\
                                                 255, 13, 166)
                

    def object_update(self):
    #更新物件資訊
        self.ball.update()
        self.obs_right.update()
        self.obs_left.update()
        self.line.update()

    def control_head(self, ID, position, speed):
    #控制頭轉動
        if ID == 1:
        #水平
            if position > MAX_HEAD_HORIZONTAL:      #超過最大值
                self.head_horizon = MAX_HEAD_HORIZONTAL
            elif position < MIN_HEAD_HORIZONTAL:    #超過最小值
                self.head_horizon = MIN_HEAD_HORIZONTAL
            else:
                self.head_horizon = position

            self.api.sendHeadMotor(ID, self.head_horizon, speed)

        elif ID == 2:
        #垂直
            if position > MAX_HEAD_VERTICAL:      #超過最大值
                self.head_vertical = MAX_HEAD_VERTICAL
            elif position < MIN_HEAD_VERTICAL:    #超過最小值
                self.head_vertical = MIN_HEAD_VERTICAL
            else:
                self.head_vertical = position

            self.api.sendHeadMotor(ID, self.head_vertical, speed)

    def control_walkinggait(self, forward, translation, theta, add_forward, add_translation, add_theta):
    #控制步態參數
        if abs(self.now_forward - forward) < add_forward:
            self.now_forward = forward
        else:
            if self.now_forward < forward:
                self.now_forward += add_forward
            elif self.now_forward > forward:
                self.now_forward -= add_forward
            else:
                self.now_forward = forward

        if abs(self.now_translation - translation) < add_translation:
            self.now_translation = translation
        else:
            if self.now_translation < translation:
                self.now_translation += add_translation
            elif self.now_translation > translation:
                self.now_translation -= add_translation
            else:
                self.now_translation = translation

        if abs(self.now_theta - theta) < add_theta:
            self.now_theta = theta
        else:
            if self.now_theta < theta:
                self.now_theta += add_theta
            elif self.now_theta > theta:
                self.now_theta -= add_theta
            else:
                self.now_theta = theta
        self.api.sendContinuousValue(self.now_forward, self.now_translation, 0, self.now_theta, 0)

    def search_ball(self, right_max = 2048-600, left_max = 2048+600, up_max = 2048, down_max = 2048-300 , scale = 30):
    #找球 右->下->左->上
        if self.search == 'right':
            self.control_head(1, self.head_horizon, 100)
            self.head_horizon -= scale
            if self.head_horizon < right_max:
                self.head_horizon = right_max
                self.search = 'down'

        elif self.search == 'down':
            self.control_head(2, self.head_vertical, 100)
            self.head_vertical -= scale
            if self.head_vertical < down_max:
                self.head_vertical = down_max
                self.search = 'left'

        elif self.search == 'left':
            self.control_head(1, self.head_horizon, 100)
            self.head_horizon += scale
            if self.head_horizon > left_max:
                self.head_horizon = left_max
                self.search = 'up'

        elif self.search == 'up':
            self.control_head(2, self.head_vertical, 100)
            self.head_vertical += scale
            if self.head_vertical > up_max:
                self.head_vertical = up_max
                self.search = 'right'
    
    def trace_ball(self, x_target, y_target):
    #追蹤目標物
        if x_target != 0 and y_target != 0:
            self.x_differ = x_target - 160
            self.y_differ = y_target - 120
            #--角度測量--#
            ##根據每個像素點和中心點的距離計算對應的角度(依機器人的高度去測量比例)
            self.x_degree = self.x_differ * (X_RATIO / 320)
            self.y_degree = self.y_differ * (Y_RATIO / 240)
            #------------#
            self.head_horizon  -= round(self.x_degree * 4096 / 360 * 0.15)
            self.head_vertical -= round(self.y_degree * 4096 / 360 * 0.15)
            self.control_head(1, self.head_horizon, 100)
            self.control_head(2, self.head_vertical, 100)
        else:
            rospy.logwarn(f"看不到球,重新找球")

    def body_trace_rotate(self,error):
    #修正機器人和目標的角度
        rotate_error = self.head_horizon - HEAD_HORIZONTAL
        if rotate_error > error:
        #左轉修正
            self.control_walkinggait(FORWARD[2], TRANSLATION[2], THETA[1], 100, 500,1)
            rospy.logdebug("左轉")
            return False
        elif rotate_error < -error:
        #右轉修正
            self.control_walkinggait(FORWARD[2], TRANSLATION[2], THETA[3], 100, 500,1)
            rospy.logdebug("右轉")
            return False
        return True

    def body_trace_straight(self,goal_degree,error):
    #修正機器人與目標的直線距離
        distance_error = self.head_vertical - goal_degree
        if distance_error > error:
            self.control_walkinggait(FORWARD[0] + CORRECT[0],\
                                     TRANSLATION[2] + CORRECT[1],\
                                     THETA[2] + CORRECT[2],\
                                     300, 200,1)
            return False
        elif distance_error < -error:
            self.control_walkinggait(FORWARD[4] + CORRECT[0],\
                                     TRANSLATION[2] + CORRECT[1],\
                                     THETA[2] + CORRECT[2],\
                                     300, 200,1)
            return False
        return True

    def body_trace_translation(self,goal_degree,error):
    #修正機器人與目標的平移距離
        translation_error = self.head_horizon - goal_degree
        if translation_error > 250:
            self.control_walkinggait(FORWARD[2] + CORRECT[0],\
                                     TRANSLATION[0] + CORRECT[1],\
                                     THETA[2] + CORRECT[2]-2,\
                                     500, 200,1)
            return False
        elif translation_error < -error:
            self.control_walkinggait(FORWARD[2] + CORRECT[0],\
                                     TRANSLATION[4] + CORRECT[1],\
                                     THETA[2] + CORRECT[2] - 1, \
                                     500, 200,1)
            return False
        elif translation_error > error:
            self.control_walkinggait(FORWARD[2] + CORRECT[0] - 200,\
                                     TRANSLATION[1] + CORRECT[1],\
                                     THETA[2] + CORRECT[2],\
                                     500, 500,1)
            return False
        elif translation_error < -error:
            self.control_walkinggait(FORWARD[2] + CORRECT[0],\
                                     TRANSLATION[3] + CORRECT[1],\
                                     THETA[2] + CORRECT[2],\
                                     500, 500,1)
            return False
        return True

    def imu_reset(self,fix,error_range):
    #讓身體imu回正，面向(回正)一開始方向
        yaw_error = self.api.imu_value_Yaw - fix 
        if abs(yaw_error) >=error_range + 10:
            if yaw_error > 0:#右修
                self.control_walkinggait(FORWARD[3] + CORRECT[0],\
                                         TRANSLATION[2] + CORRECT[1],\
                                         THETA[4] + CORRECT[2],\
                                         200,500,1)
                rospy.logdebug("大右轉")
            elif yaw_error < 0:#左修
                self.control_walkinggait(FORWARD[3] + CORRECT[0],\
                                         TRANSLATION[2] + CORRECT[1],\
                                         THETA[0] + CORRECT[2],\
                                         200,500,1)
                rospy.logdebug("大左轉")
            return False 
        elif abs(yaw_error) >=error_range:
            if yaw_error > 0:#右修
                self.control_walkinggait(FORWARD[2] + CORRECT[0],\
                                         TRANSLATION[2] + CORRECT[1],\
                                         THETA[3] + CORRECT[2],\
                                         200,500,1)
                rospy.logdebug("右轉")
            elif yaw_error < 0:#左修
                self.control_walkinggait(FORWARD[2] + CORRECT[0],\
                                         TRANSLATION[2] + CORRECT[1],\
                                         THETA[1] + CORRECT[2],\
                                         200,500,1)
                rospy.logdebug("左轉")
            return False
        return True
        
    def obs_dodge(self):
        if self.ball.target_size < 3000: #要測
            if self.direction == 'left' and not self.dodging_obs:
                if abs(self.obs_left.edge_max.x-self.ball.center.x) < 8: #要測
                    if self.obs_left.edge_max.x < self.ball.center.x:
                        self.api.sendContinuousValue(0+CORRECT[0], TRANSLATION[4]+CORRECT[1], 0, 0+CORRECT[2], 0)
                        rospy.logwarn("第一階段,右平移避障")
                        rospy.sleep(5)
                        self.dodging_obs = True
                        return False
                    return True
                    
            elif self.direction == 'right' and not self.dodging_obs:
                if abs(self.obs_right.edge_min.x-self.ball.center.x) < 8:
                    if self.obs_right.edge_min.x > self.ball.center.x:
                        self.api.sendContinuousValue(0+CORRECT[0], TRANSLATION[0]+CORRECT[1], 0, 0+CORRECT[2], 0)
                        rospy.logwarn("第一階段,左平移避障")
                        rospy.sleep(5)
                        self.dodging_obs = True
                        return False
                    return True
            rospy.logdebug("straight~")
            return True
        else:
            if self.obs_second < False:
                self.dodging_obs = False
                self.obs_second = True
            if self.direction == 'left' and not self.dodging_obs:
                if abs(self.obs_right.edge_min.x-self.ball.center.x) < 8:
                    if self.obs_right.edge_min.x > self.ball.center.x:
                        self.api.sendContinuousValue(0+CORRECT[0], TRANSLATION[0]+CORRECT[1], 0, 0+CORRECT[2], 0)
                        rospy.logwarn("第二階段,左平移避障")
                        rospy.sleep(3)
                        self.dodging_obs = True
                        return False
                    return True
                    
            elif self.direction == 'right' and not self.dodging_obs:
                if abs(self.obs_left.edge_max.x-self.ball.center.x) < 8:
                    if self.obs_left.edge_max.x < self.ball.center.x:
                        self.api.sendContinuousValue(0+CORRECT[0], TRANSLATION[4]+CORRECT[1], 0, 0+CORRECT[2], 0)
                        rospy.logwarn("第二階段,右平移避障")
                        rospy.sleep(3)
                        self.dodging_obs = True
                        return False
                    return True
                    
            rospy.logdebug("straight~")
            return True
            
def strategy(): 
    rospy.init_node('PK_strategy', anonymous=True, log_level=rospy.DEBUG)   #初始化node
    send = Sendmessage()
    pk = PenaltyKick(send)
    walk_stop = True
    check_find_ball = 0
    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        if send.is_start:
            pk.object_update()
            pk.drawImageFunction('start')
            if pk.state != 'defender':
                
                if walk_stop and pk.search_finish:
                    send.sendBodySector(30)
                    rospy.sleep(2)
                    send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                    walk_stop = False

                if pk.state == 'search_ball':
                    if not pk.ball.get_target:
                        pk.search_ball(#right_max = 2048-600,\
                                    #left_max = 2048+600,\
                                    #up_max = 2048,\
                                    #down_max = 2048-600,\
                                    scale = 50)
                    else:
                        if pk.trace_first:
                            pk.state = 'trace_ball_first'
                        else:
                            pk.state = 'watch_ball'         
                
                elif pk.state == 'trace_ball_first':
                    pk.trace_ball(pk.ball.center.x,pk.ball.center.y)
                    if pk.ball.target_size > 5400:
                        pk.kick_count = pk.kick_count - 1 
                    elif pk.imu_reset(0,5):
                        if pk.body_trace_translation(pk.location_y,HEAD_ERROR_RANGE_Y):      
                            if pk.body_trace_straight(pk.location_x,HEAD_ERROR_RANGE_X):
                                pk.kick_count = pk.kick_count - 1 
                    else:
                            pk.kick_count = 2

                    if pk.kick_count == 0:
                        send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                        rospy.sleep(2)
                        send.sendBodySector(29)
                        rospy.sleep(2)
                        if pk.direction == 'left':
                            send.sendBodySector(7911)
                            pk.location_y = KICK_DEGREE_HORIZONTAL_R #頭水平轉的位置
                            rospy.loginfo("右踢")
                        elif pk.direction == 'right':
                            send.sendBodySector(7933)
                            pk.location_y = KICK_DEGREE_HORIZONTAL_L #頭水平轉的位置
                            pk.search = 'left'
                            rospy.loginfo("左踢")
                        rospy.sleep(4)
                        send.sendBodySector(29)
                        rospy.sleep(3)
                        walk_stop          = True
                        pk.search_finish   = False
                        pk.trace_first     = False
                        pk.head_vertical   = 1848
                        pk.kick_count      = 2
                        pk.now_forward     = 0 
                        pk.now_translation = 0
                        pk.now_theta       = 0 
                        pk.state = 'search_ball'

                elif pk.state == 'watch_ball':
                    pk.trace_ball(pk.ball.center.x,pk.ball.center.y)
                    if abs(pk.ball.center.x) - 160 < 4 and abs(pk.ball.center.y) - 120 < 3:
                        pk.shoting_angle = pk.head_horizon
                        rospy.sleep(1)
                        pk.search_finish   = True
                        pk.state = 'trace_ball_second'
                
                elif pk.state == 'trace_ball_second':
                    pk.trace_ball(pk.ball.center.x,pk.ball.center.y)

                    if pk.obs_dodge():
                        if pk.body_trace_rotate(ROTATE_ERROR):
                            pk.now_theta = 0
                            if pk.body_trace_straight(SHOT_DEGREE,HEAD_ERROR_RANGE_X):
                                pk.state = 'back_on_track'

                elif pk.state == 'back_on_track':
                    pk.trace_ball(pk.ball.center.x,pk.ball.center.y)
                    if pk.direction == 'left':
                        if pk.imu_reset(-5,5):
                            pk.state = 'shoting'
                    elif pk.direction == 'right':
                        if pk.imu_reset(5,5):
                            pk.state = 'shoting'

                elif pk.state == 'shoting':
                    pk.trace_ball(pk.ball.center.x,pk.ball.center.y)
                    if pk.ball.target_size > 6000:
                        pk.kick_count = pk.kick_count - 1 
                    elif pk.imu_reset(0,5):
                        if pk.body_trace_translation(pk.location_y,HEAD_ERROR_RANGE_Y):
                            if pk.body_trace_straight(pk.location_x+50,HEAD_ERROR_RANGE_X):
                                pk.kick_count = pk.kick_count - 1 
                    else:
                            pk.kick_count = 2

                    if pk.kick_count == 0:
                        send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                        rospy.sleep(2)
                        if pk.direction == 'left':
                            send.sendBodySector(7933)
                            rospy.loginfo("左踢")
                        elif pk.direction == 'right':
                            send.sendBodySector(7911)
                            rospy.loginfo("右踢")
                        walk_stop          = True
                        pk.search_finish   = False
                        pk.state = 'finish_shoting'

            else:
                rospy.loginfo(pk.ball.target_size)
                if not pk.ball.get_target and not pk.defence:
                        pk.search_ball(#right_max = 2048-600,\
                                    #left_max = 2048+600,\
                                    #up_max = 2048,\
                                    #down_max = 2048-600,\
                                    scale = 30)
                elif pk.defence:
                    rospy.logwarn("防禦中")
                else:
                    pk.trace_ball(pk.ball.center.x,pk.ball.center.y)
                    if pk.ball.target_size > 2080 and check_find_ball < 8:  #要測
                        check_find_ball = check_find_ball +1
                    elif check_find_ball >= 8:
                        send.sendBodySector(39)         #給定蹲下速度
                        # send.sendWalkParameter('send',1,\
                        #             stand_height = 42.3)
                        pk.defence = True
                    else:
                        check_find_ball = 0
        else:
            rospy.logdebug('strategy close')
            if TEST:
                pk.object_update()
                print(pk.ball.target_size)
            if not walk_stop:
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                rospy.sleep(2)
                send.sendBodySector(29)
            pk.init()
            walk_stop = True
            check_find_ball = 0
        r.sleep()

if __name__ == '__main__':
    try:
       strategy()
    except rospy.ROSInterruptException:
        pass
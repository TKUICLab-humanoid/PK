#!/usr/bin/env python
# coding=utf-8
from cmath import sqrt
from re import S, T
from traceback import print_tb
 
import numpy as np
import rospy
from Python_API import Sendmessage

#--影像中像素點對到中心對於頭部刻度的角度比例--#
X_RATIO                    = 64.5
Y_RATIO                    = 40
#------------------------------------------#
#                            [大前進,前進, 原地,   後退,大後退]
FORWARD                    = [3000, 1000, -500, -1000, -1500]
#                            [大左移,左移, 原地,   右移,大右移]
TRANSLATION                = [1500, 1000,    0, -1000, -1500]
#                            [大左旋,左旋, 原地,   右旋,大右旋]
THETA                      = [   7,    3,    0,    -3,    -7]
#
KICK_DEGREE , KICK_ERROR   = 1275, 20
SHOT_DEGREE , SHOT_ERROR   = 1375, 20
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

    def __init__(self, color, object_type,api):

        self.api              = api
        self.color            = self.color_dict[color]
        self.edge_max         = Coordinate(0, 0)
        self.edge_min         = Coordinate(0, 0)
        self.center           = Coordinate(0, 0)
        self.get_target       = False
        self.target_size      = 0

        update_strategy = { 'OBS'     : self.get_obs_object,
                            'Line'    : self.get_line_object,
                            'Ball'    : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_obs_object(self):
        max_object_size = max(self.api.color_mask_subject_size[self.color])
        max_object_idx = self.api.color_mask_subject_size[self.color].index(max_object_size)
        return max_object_idx if max_object_size > 5000 else None

    def get_line_object(self):
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
        self.obs        = ObjectInfo( "Blue"  , 'OBS', self.api) 
        self.line       = ObjectInfo( "White" , 'Line', self.api)
        self.direction  = 'left'
        self.location_x = 1748 
        self.init()
        
    def init(self):
    #初始化
        #重置旗標
        self.state           = 'search_ball'
        self.search          = 'right'
        self.trace_first     = True
        #重置頭位置
        self.head_horizon    = HEAD_HORIZONTAL
        self.head_vertical   = HEAD_VERTICAL
        self.control_head(1, self.head_horizon, 30) 
        self.control_head(2, self.head_vertical, 30)
        #步態
        self.walkinggait     = 'stop'  
        self.now_forward     = 0 
        self.now_translation = 0
        self.now_theta       = 0 

        self.count           = 3
        self.drawImageFunction('init')
        self.api.sendSensorReset(1,1,1)
        if self.api.DIOValue == 0x09 or self.api.DIOValue == 0x19:
            self.direction  = 'right'
            self.location_x = 2288                  #頭水平轉的位置
        elif self.api.DIOValue == 0x0a or self.api.DIOValue == 0x1a:
            self.direction  = 'straight'
            self.location_x = 2048                  #頭水平轉的位置
        elif self.api.DIOValue == 0x08 or self.api.DIOValue == 0x18:
            self.direction  = 'left'
            self.location_x = 1808                  #頭水平轉的位置
        self.location_y     = KICK_DEGREE           #頭垂直轉的位置 

    def drawImageFunction(self,state):
    #繪圖
        if DRAW_FUNCTION_FLAG:
            if state == 'init':
                self.api.drawImageFunction(1, 1, 0, 0, 0, 0, 255, 0, 255)
                self.api.drawImageFunction(2, 0, 0, 0, 0, 0, 0, 255, 0)
                self.api.drawImageFunction(3, 0, 0, 0, 0, 0, 0, 255, 0)
            elif state == 'start':
                self.api.drawImageFunction(1, 1, self.ball.edge_min.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_max.y,\
                                                 255, 0, 255)
                self.api.drawImageFunction(2, 0, 160, 160, 110, 130, 255, 255, 255)
                self.api.drawImageFunction(3, 0, 150, 170, 120, 120, 255, 255, 255)

    def object_update(self):
    #更新物件資訊
        self.ball.update()
        self.obs.update()
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
            return False
        elif rotate_error < -error:
        #右轉修正
            self.control_walkinggait(FORWARD[2], TRANSLATION[2], THETA[3], 100, 500,1)
            return False
        else:
            return True

    def body_trace_straight(self,goal_degree,error):
    #修正機器人與目標的距離
        distance_error = self.head_vertical - goal_degree
        if distance_error > error:
            self.control_walkinggait(FORWARD[0], TRANSLATION[2], THETA[2], 500, 500,1)
            return False
        elif distance_error < -error:
            self.control_walkinggait(FORWARD[4], TRANSLATION[2], THETA[2], 500, 500,1)
            return False
        else:
            return True
    
    def imu_reset(self,fix,error_range):
    #讓身體imu回正，面向(回正)一開始方向
        yaw_error = self.api.imu_value_Yaw - fix 
        if abs(yaw_error) >=error_range:
            if yaw_error > 0:#右修
                self.control_walkinggait(FORWARD[3],TRANSLATION[2],THETA[3],500,500,1)
                rospy.logdebug("右轉")
            elif yaw_error < 0:#左修
                self.control_walkinggait(FORWARD[3],TRANSLATION[2],THETA[1],500,500,1)
                rospy.logdebug("左轉")
            return False
        else:
            return True

def strategy():
    rospy.init_node('PK_strategy', anonymous=True, log_level=rospy.DEBUG)   #初始化node
    send = Sendmessage()
    pk = PenaltyKick(send)
    walk_stop = True
    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        if send.is_start:
            pk.object_update()
            pk.drawImageFunction('start')
            rospy.loginfo(f"now state:{pk.state}")
            rospy.loginfo(f"目標佔位:{pk.direction}")
            rospy.loginfo(f"水平刻度:{pk.head_horizon},垂直刻度:{pk.head_vertical}")
            rospy.loginfo(f"前後:{pk.now_forward},平移:{pk.now_translation},旋轉:{pk.now_theta}")
            if walk_stop:
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
                if abs(pk.head_vertical - pk.location_y) <= KICK_ERROR:
                    pk.count = pk.count - 1 
                elif not pk.imu_reset(0,5):
                    pk.count = 2
                elif   pk.direction == 'right':
                    if pk.head_horizon < pk.location_x:
                        translation_y = TRANSLATION[4]
                    elif pk.head_horizon >= pk.location_x:
                        translation_y = TRANSLATION[0]
                        
                    if pk.head_vertical > pk.location_y:
                        forward_x = FORWARD[0]
                    elif pk.head_vertical <= pk.location_y:
                        forward_x = FORWARD[3]
                    pk.control_walkinggait(forward_x,translation_y,0,200,200,1)
                    pk.count = 2

                elif pk.direction == 'left':
                    if pk.head_horizon > pk.location_x:
                        translation_y = TRANSLATION[0]
                    elif pk.head_horizon <= pk.location_x:
                        translation_y = TRANSLATION[4]

                    if pk.head_vertical > pk.location_y:
                        forward_x = FORWARD[0]
                    elif pk.head_vertical <= pk.location_y:
                        forward_x = FORWARD[3]
                    pk.control_walkinggait(forward_x,translation_y,0,200,200,1)
                    pk.count = 2

                if pk.count == 0:
                    send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                    rospy.sleep(2)
                    if pk.direction == 'left':
                        send.sendBodySector(7911)   #7934
                        rospy.loginfo("右踢")
                    elif pk.direction == 'right':
                        send.sendBodySector(7933)
                        rospy.loginfo("左踢")
                    rospy.sleep(4)
                    send.sendBodySector(29)
                    rospy.sleep(3)
                    pk.trace_first     = False
                    pk.head_vertical   = 2048
                    pk.now_forward     = 0 
                    pk.now_translation = 0
                    pk.now_theta       = 0 
                    pk.state = 'search_ball'

            elif pk.state == 'watch_ball':
                pk.trace_ball(pk.ball.center.x,pk.ball.center.y)
                if abs(pk.ball.center.x) - 160 < 8 and abs(pk.ball.center.y) - 120 < 6:
                    rospy.sleep(1)
                    walk_stop = True
                    pk.state = 'trace_ball_second'
            
            elif pk.state == 'trace_ball_second':
                pk.trace_ball(pk.ball.center.x,pk.ball.center.y)
                if pk.body_trace_rotate(ROTATE_ERROR):
                    pk.now_theta = 0
                    if pk.body_trace_straight(SHOT_DEGREE,SHOT_ERROR):
                        pk.state = 'back_on_track'
            
            elif pk.state == 'back_on_track':
                pk.trace_ball(pk.ball.center.x,pk.ball.center.y)
                if pk.imu_reset(0,5):
                    pk.state = 'shoting'

            elif pk.state == 'shoting':
                pass

        else:
            rospy.logdebug('strategy close')
            if not walk_stop:
                send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                rospy.sleep(2)
                send.sendBodySector(29)
            pk.init()
            walk_stop = True
        r.sleep()

if __name__ == '__main__':
    try:
       strategy()
    except rospy.ROSInterruptException:
        pass
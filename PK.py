#!/usr/bin/env python
# coding=utf-8
from cmath import sqrt
from re import S, T
import time
from traceback import print_tb
 
import numpy as np
import rospy
from Python_API import Sendmessage
send = Sendmessage()

CORRECT = [-1100, 0, 0]    
LEVEL_LEFT_CORRECT = [-1300, 2500,1]
LEVEL_RIGHT_CORRECT = [-1100, -2500,0] 
LEFT_CORRECT = [-1300, 0, 3] 
RIGHT_CORRECT = [-1100, 0, -3] 

  #預先讓頭轉至大概角度
SLOWDOWN = [0,0]
ROTATE_MISTAKE = 50  # x , y , theta
KICK_DEGREE , KICK_ERROR = 2840, 20# 第一次小踢   # left 2840 right 2830
KICK_DEGREE2 , KICK_ERROR2 = 2690, 20 #直接射門
KICK_DEGREE_MISTAKE = 50

LOOK_BALL = 1488

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

    def __init__(self, color, object_type):

        self.color            = self.color_dict[color]
        self.edge_max         = Coordinate(0, 0)
        self.edge_min         = Coordinate(0, 0)
        self.center           = Coordinate(0, 0)
        self.get_target       = False
        self.target_size      = 0

        update_strategy = { 'OBS_blue': self.get_object,
                            'Line'    : self.get_line_object,
                            'Ball'    : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_object(self):
        max_object_size = max(send.color_mask_subject_size[self.color])
        max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)
        return max_object_idx if max_object_size > 10000 else None

    def get_line_object(self):
        max_object_size = max(send.color_mask_subject_size[self.color])
        max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)
        return max_object_idx if max_object_size > 2000 else None

    def get_ball_object(self):
        object_idx = None
        for i in range(send.color_mask_subject_cnts[self.color]):
            length_width_diff = abs(abs(send.color_mask_subject_XMax[self.color][i] - send.color_mask_subject_XMin[self.color][i]) - abs(send.color_mask_subject_YMax[self.color][i] - send.color_mask_subject_YMin[self.color][i]))
            if 100 < send.color_mask_subject_size[self.color][i] < 2500 and length_width_diff < 8:
                object_idx = i
        
        return object_idx

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target  = True
            self.edge_max.x  = send.color_mask_subject_XMax[self.color][object_idx]
            self.edge_min.x  = send.color_mask_subject_XMin[self.color][object_idx]
            self.edge_max.y  = send.color_mask_subject_YMax[self.color][object_idx]
            self.edge_min.y  = send.color_mask_subject_YMin[self.color][object_idx]
            self.center.x    = send.color_mask_subject_X[self.color][object_idx]
            self.center.y    = send.color_mask_subject_Y[self.color][object_idx]
            self.target_size = send.color_mask_subject_size[self.color][object_idx]

            # rospy.loginfo(self.target_size)
            # rospy.logdebug(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
        else:
            self.get_target = False

class MotorMove():

    def __init__(self):
        self.head_horizon   = 2048     # 頭部水平刻度
        self.head_vertical  = 2048    # 頭部垂直刻度
        self.head_horizon_flag = 0   # 找目標物的旗標
        self.x_differ = 0            # 目標與中心x差距
        self.y_differ = 0            # 目標與中心y差距
        self.x_degree = 0            # 目標與中心x角度
        self.y_degree = 0            # 目標與中心y角度
        self.body_level = 0          
        self.body_rotate = 0         # 身體要旋轉多少？？？
        self.body_straight = 0
        self.now_state = 0          # 當前執行步驟 
        self.Move_waist = 0         # 修腰
        self.NowX = 0               # 現在要移動的x量
        self.NowY = 0               # 現在要移動的x量
        self.NowTheta = 0     
        self.step_jump = False
        self.cnt = 3

    def move_head(self, ID, Position, max_head_horizon_size, max_head_vertical_size, Speed):#轉動頭之刻度
        if ID == 1:
            if abs(Position - 2048) > max_head_horizon_size: 
                if (Position - 2048) >= 0:
                    Position = 2048 + max_head_horizon_size
                elif (Position - 2048) <= 0:
                    Position = 2048 - max_head_horizon_size
                self.head_horizon = Position
                send.sendHeadMotor(ID, Position, Speed)
            else :
                self.head_horizon = Position
                send.sendHeadMotor(ID, Position, Speed)

        elif ID == 2:
            if abs(Position - 2048) > max_head_vertical_size:
                if (Position - 2048) < 0:
                    Position = 2048 + max_head_vertical_size
                elif (Position - 2048) > 0:
                    Position = 2048 - max_head_vertical_size
                self.head_vertical = Position
                send.sendHeadMotor(ID, Position, Speed)
            else:
                self.head_vertical = Position
                send.sendHeadMotor(ID, Position, Speed)
                
    def view_move(self, right_place, left_place,up_place ,down_place, speed, delay):       #搜尋目標物，掃描畫面
        if self.head_horizon_flag == 1:
            if self.head_horizon >= left_place:
                self.move_head(1, self.head_horizon, 880, 880, speed)
                self.head_horizon = self.head_horizon - speed
                time.sleep(delay)
            elif self.head_vertical >= down_place:
                self.move_head(2, self.head_vertical, 880, 880, 100)
                self.head_vertical = self.head_vertical - speed
                time.sleep(delay)
            else:
                self.head_horizon_flag = 0
        else:
            if self.head_horizon <= right_place:
                self.move_head(1, self.head_horizon, 880, 880, speed)
                self.head_horizon = self.head_horizon + speed
                time.sleep(delay)
            elif self.head_vertical <= up_place:
                self.move_head(2, self.head_vertical, 880, 880, 100)
                self.head_vertical = self.head_vertical + speed
                time.sleep(delay)
            else:
                self.head_horizon_flag = 1

    def trace_revise(self, x_target, y_target, speed):                                      #追蹤目標物
        self.x_differ = x_target - 160
        self.y_differ = y_target - 120
        #--角度測量--#
        ##根據每個像素點和中心點的距離計算對應的角度(依機器人的高度去測量比例)
        self.x_degree = self.x_differ * (64.5 / 320)
        self.y_degree = self.y_differ * (40 / 240) # 70.42 43.3 要重新測
        #------------#
        self.move_head(1, self.head_horizon - round(self.x_degree * 4096 / 360 * 0.15), 880, 880, speed)
        self.move_head(2, self.head_vertical - round(self.y_degree * 4096 / 360 * 0.15), 800, 800, speed)

    def body_trace_rotate(self, spot_degree, error):                                        #機器人要旋轉到的角度 , 誤差 
        self.body_rotate = self.head_horizon - spot_degree
        if self.body_rotate > error:
            self.MoveContinuous(LEFT_CORRECT[0], LEFT_CORRECT[1], LEFT_CORRECT[2], 500, 500, 1)
            rospy.loginfo("go left  ")
            time.sleep(0.05)
        elif self.body_rotate < -error:
            self.MoveContinuous(RIGHT_CORRECT[0], RIGHT_CORRECT[1], RIGHT_CORRECT[2], 500, 500, 1)
            rospy.loginfo("go right  ")
            time.sleep(0.05)

    def body_trace_level(self, degree_level, degree_straight, error_level, error_straight): #要的水平角度  要的直走角度 容忍的水平角度誤差 容忍的直走角度
        self.body_level = self.head_horizon - degree_level
        self.body_straight = self.head_vertical - degree_straight
        rospy.loginfo(self.body_level,self.body_straight)
        if abs(self.body_level) > error_level:
            if self.body_level > 0:  # 左正右負
                rospy.loginfo("go left level")
            elif self.body_level < 0:
                rospy.loginfo("go right level")
        elif abs(self.body_straight) > error_straight:
            if self.body_straight > 0:
                rospy.loginfo("go back")
            elif self.body_straight < 0:
                rospy.loginfo("go ahead")
        else:
            self.step_jump = True

    def body_trace_straight(self, spot_degree, error):                                      #直走到達目標的點,誤差
            if (self.head_vertical - spot_degree) < -error:
                self.MoveContinuous(4000 + CORRECT[0], 0 + CORRECT[1], 0 + CORRECT[2], 500 , 500, 1)
                rospy.loginfo("go ahead %d", self.head_vertical)
            elif (self.head_vertical - spot_degree) > error:
                self.MoveContinuous(-900 + CORRECT[0], 0 + CORRECT[1], 0 + CORRECT[2], 500 , 500, 1)
                rospy.loginfo("go back %d", self.head_vertical)
            
            elif abs(self.head_vertical - spot_degree) <= error:
                self.step_jump = True

    def MoveContinuous(self, ExpX, ExpY, ExpTheta, AddX, AddY, AddTheta):                   #慢慢減動作，避免一次到位而不穩定
        if abs(self.NowX - ExpX) < AddX:
            self.NowX = ExpX
        else:
            if self.NowX < ExpX:
                self.NowX += AddX
            elif self.NowX > ExpX:
                self.NowX -= AddX
            else:
                pass

        if abs(self.NowY - ExpY) < AddY:
            self.NowY = ExpY
        else:
            if self.NowY < ExpY:
                self.NowY += AddY
            elif self.NowY > ExpY:
                self.NowY -= AddY
            else:
                pass

        if abs(self.NowTheta - ExpTheta) < AddTheta:
            self.NowTheta = ExpTheta
        else:
            if self.NowTheta < ExpTheta:
                self.NowTheta += AddTheta
            elif self.NowTheta > ExpTheta:
                self.NowTheta -= AddTheta
            else:
                pass
        send.sendContinuousValue(self.NowX, self.NowY, 0, self.NowTheta, 0)

    def bodyauto_close(self, next_state):                                                   #reset機器人狀態，控制步態開關
        if self.now_state == next_state:
            pass
        elif self.now_state != next_state:
            send.sendBodyAuto(0,0,0,0,1,0)
            time.sleep(1)
            self.now_state = next_state

class PenaltyKick():
    def __init__(self):
        self.pre_state = 'None'
        self.init()
        self.ball = ObjectInfo( 0,'Ball')     # 0 為橘色
        self.obs  = ObjectInfo( 2,'OBS_blue') # 2 為橘色
        self.line = ObjectInfo( 6,'Line')     # 6 為白色
    def init(self):
        self.state = 'search_ball'
        send.sendHeadMotor(1, 2048, 30)  # reset head
        send.sendHeadMotor(2, 2248, 30)
        if self.pre_state != 'None':
            time.sleep(1.5)
            send.sendBodySector(29)  # reset body
            self.pre_state = 'None'
        send.sendSensorReset(1,1,1)
        if send.DIOValue == 17 or send.DIOValue == 25:
            self.direction = 'right'
        else:
            self.direction = 'left'
    
    def main(self):

        if self.state == 'search_ball':
        ##找球
            self.step_search_ball(2248, 1848, 2048, 1800)
            if motor.step_jump:
                self.state  = "start_trace_ball"
                motor.step_jump = False

        elif self.state == 'start_trace_ball':  
        ## 持續盯著球
            self.step_start_trace_ball()

        elif self.state == "walk_ball":
            self.step_walk_ball(190,self.direction)
            if motor.step_jump:
                self.state = "trace_ball"
                motor.step_jump = False
                motor.cnt = 3

        elif self.state == 'trace_ball':
            self.step_kick_trace_ball()

        elif self.state == "avoid_obs":
            self.step_avoid_obs(150)

        elif self.state == "kick_search_ball":#小踢完後收尋球確認球的位置
            self.step_search_ball(2548, 1548, 2048, 1800)
            if motor.step_jump:
                self.state = "walk_to_ball"
                motor.step_jump = False

        elif self.state =="walk_to_ball":
            self.walk_to_ball()
            if motor.step_jump:
                self.state = "kick.imu_reset"
                motor.step_jump = False

        elif self.state == "kick.imu_reset":
            self.step_imu_reset_obs()
            if motor.step_jump:
                motor.move_head(2,1800, 880, 880, 50)
                motor.move_head(1,2048, 880, 880, 50)
                ##這裡要再更改##
                if (self.ball.edge_max.x <= 240):
                    self.direction = "left"
                else :
                    self.direction = "right"
                rospy.loginfo(f"way to score{self.ball.edge_max.x},{self.direction}")
                motor.MoveContinuous(CORRECT[0], CORRECT[1], CORRECT[2], 500, 500, 1)
                time.sleep(1)
                self.state = "kick.move_to_ball"
                motor.step_jump = False
            
        elif self.state == "kick.move_to_ball":
            self.step_walk_ball(160,self.direction)
            time.sleep(0.02)
            if motor.step_jump :
                motor.cnt = 3 
                time.sleep(0.02)
                self.state = "kick.goahead_to_ball"
                motor.step_jump = False

        elif self.state == "kick.goahead_to_ball":
            self.step_score_ahead(2830)
            if motor.step_jump :
                motor.bodyauto_close(0)
                time.sleep(1.3)
                if self.direction == "right":
                    send.sendBodySector(7911)
                    rospy.loginfo(f"way to score 7911")
                else:
                    send.sendBodySector(7933)
                    rospy.loginfo(f"way to score 7933")
                self.state = "finish"
                motor.step_jump = False

    def step_search_ball(self,right,left,up,down):
    #找球
        if self.ball.target_size < 800:  # 500要測
            motor.view_move(right, left, up, down, 70, 0.05)
            time.sleep(0.05)
            
        elif self.ball.target_size > 800:
            time.sleep(0.1)
            motor.step_jump = True

    def step_start_trace_ball(self):
    #找到球,持續讓球維持在螢幕正中心
        if abs(self.ball.center.x - 160) > 8 or abs(self.ball.center.y - 120) > 6:
            
            motor.trace_revise(self.ball.center.x, self.ball.center.y, 50)
            time.sleep(0.05)
        else:
            rospy.loginfo(f"motor.head_horizon{motor.head_horizon}")
            time.sleep(0.05)
            motor.move_head(1, 2048, 880, 880, 50)
            motor.move_head(2, 2770, 880, 880, 50)
            time.sleep(0.05)
            motor.bodyauto_close(1)
            motor.MoveContinuous(CORRECT[0], CORRECT[1], CORRECT[2], 500, 500, 1)  # 原地踏步
            time.sleep(2)
            rospy.loginfo("find left left left")
            self.state  = "walk_ball"

    def step_walk_ball(self,bound,direction):
    #根據一開始的方向決定往球的左或右邊走
        send.drawImageFunction(8, 0, bound, bound, 0, 320, 255, 255, 255)  # 對球中心線
        if abs(send.imu_value_Yaw) > 5:
            self.imu_reset(0, 5)
            rospy.loginfo("reset ")
            motor.cnt = 3
        elif motor.cnt > 0:
            if direction == 'left':
                if self.ball.center.x_min < bound:
                    rospy.loginfo("go left")
                    motor.MoveContinuous(LEVEL_LEFT_CORRECT[0] - SLOWDOWN[0], LEVEL_LEFT_CORRECT[1] - SLOWDOWN[1],LEVEL_LEFT_CORRECT[2], 500, 500, 1)
                    motor.cnt= 3  # 要讓球最小值離開過三次界線才跳出 預防步態不穩
                else:
                    motor.cnt -= 1
            elif direction == 'right':
                if self.ball.center.x_min > bound or self.ball.center.x_min == 0:
                    rospy.loginfo("go right")
                    motor.MoveContinuous(LEVEL_RIGHT_CORRECT[0] - SLOWDOWN[0], LEVEL_RIGHT_CORRECT[1] - SLOWDOWN[1], LEVEL_RIGHT_CORRECT[2], 500, 500, 1)
                    motor.cnt = 3  # 要讓球最小值離開過三次界線才跳出 預防步態不穩
                else:
                    motor.cnt -= 1
        else:
            motor.cnt = 2
            time.sleep(0.2)
            motor.step_jump = True

    def step_kick_trace_ball(self):
    #走到球旁邊,到一定距離後踢球
        motor.trace_revise(self.ball.center.x, self.ball.center.y, 50)  # 追蹤球
        if abs(motor.head_vertical - KICK_DEGREE) <= KICK_ERROR and abs(send.imu_value_Yaw) <= 5:  # 跟球距離洽當就踢   20 要再側
            rospy.loginfo("small kick 1")  # 小踢
            motor.cnt = motor.cnt - 1
        elif abs(send.imu_value_Yaw) > 5:
            self.imu_reset(0, 5)
            rospy.loginfo("reset ")
            motor.cnt = 2
        elif abs(motor.head_vertical - KICK_DEGREE) > KICK_ERROR :  # 如果太遠
            if motor.head_vertical - KICK_DEGREE < KICK_ERROR:
                motor.MoveContinuous(CORRECT[0] + 3000, CORRECT[1], CORRECT[2], 500, 500, 1)
                rospy.loginfo("前進")
                motor.cnt = 2
            elif motor.head_vertical - KICK_DEGREE > KICK_ERROR:
                motor.MoveContinuous(CORRECT[0] - 300, CORRECT[1], CORRECT[2], 500, 500, 1)
                rospy.loginfo("後退")
                motor.cnt = 2
        if motor.cnt == 0:
            motor.bodyauto_close(0)
            time.sleep(4)
            if self.direction == 'left':
                send.sendBodySector(7933)   #7934
                rospy.loginfo("右踢")
            elif self.direction == 'right':
                send.sendBodySector(7911)
                rospy.loginfo("左踢")
            time.sleep(4)
            send.sendBodySector(29)
            motor.move_head(2, LOOK_BALL, 880, 880, 50)
            motor.move_head(1, 2048, 880, 880, 50)
            time.sleep(3)
            motor.bodyauto_close(1)
            motor.MoveContinuous(CORRECT[0], CORRECT[1], CORRECT[2], 500, 500, 1) 
            self.imu_reset(0,5)
            self.state = "avoid_obs"
            motor.cnt = 5

    def step_score_ahead(self,score_vertical):
    #修正完機器人的角度,直走到球旁
        rospy.loginfo(f"ver{motor.head_vertical}")  # 小踢
        motor.trace_revise(self.ball.center.x, self.ball.center.y, 50)  # 追蹤球
        if abs(motor.head_vertical - score_vertical) <= 30:  # 跟球距離洽當就踢   20 要再側
            rospy.loginfo("small kick 2")  # 小踢
            motor.cnt = motor.cnt - 1
        elif abs(send.imu_value_Yaw) > 5:
            self.imu_reset(0, 5)
            rospy.loginfo("reset ")
            motor.cnt = 1
        elif abs(motor.head_vertical - score_vertical) > 30:  # 如果太遠
            if motor.head_vertical - score_vertical < score_vertical:
                motor.MoveContinuous(CORRECT[0] + 2200, CORRECT[1], CORRECT[2], 500, 500, 1)
                rospy.loginfo("前進")
                motor.cnt = 1
            elif motor.head_vertical - score_vertical > 30:
                motor.MoveContinuous(CORRECT[0] - 300, CORRECT[1], CORRECT[2], 500, 500, 1)
                rospy.loginfo("後退")
                motor.cnt = 1
        if motor.cnt == 0:
            motor.step_jump = True

    def step_avoid_obs(self,avoidobs):
    #踢完球,先平移避開障礙物
        send.drawImageFunction(10, 0, avoidobs, avoidobs, 0, 320, 255, 0, 255)  # 對球中心線
        rospy.loginfo("aaaa%d,%d",self.obs.edge_max.x,motor.cnt)
        if self.direction == "right":
            if abs(send.imu_value_Yaw) > 5 :
                self.imu_reset(0,5)
                motor.cnt = 2
            elif motor.cnt > 0:
                if self.obs.edge_max.x > avoidobs:
                    rospy.loginfo("go righthshfjgjgjg")  # 右平移
                    motor.MoveContinuous(LEVEL_RIGHT_CORRECT[0], LEVEL_RIGHT_CORRECT[1], LEVEL_RIGHT_CORRECT[2], 500, 500,1)
                    motor.cnt = 3  # 要讓球最小值離開過三次界線才跳出 預防步態不穩
                else:
                    motor.cnt -= 1
            else:
                motor.cnt = 2
                time.sleep(0.2)
                motor.move_head(1, 1900, 880, 880, 50)
                motor.move_head(2, 2400, 880, 880, 50)
                time.sleep(0.2)
                self.state = "kick_search_ball"
        elif self.direction == "left":
            if abs(send.imu_value_Yaw) > 5 :
                self.imu_reset(0,5)
                motor.cnt = 2
            elif motor.cnt > 0:
                if self.obs.edge_min.x < avoidobs:
                    rospy.loginfo("go leftfftftftftftf")  # 左平移
                    motor.MoveContinuous(LEVEL_LEFT_CORRECT[0], LEVEL_LEFT_CORRECT[1], LEVEL_LEFT_CORRECT[2], 500, 500,1)
                    motor.cnt = 3  # 要讓球最小值離開過三次界線才跳出 預防步態不穩
                else:
                    motor.cnt -= 1
            else:
                motor.cnt = 2
                time.sleep(0.2)
                motor.move_head(1, 2200, 880, 880, 50)
                motor.move_head(2, 2400, 880, 880, 50)
                time.sleep(0.2)
                self.state = "kick_search_ball"

    def walk_to_ball(self):
    #踢完球,走到球旁
        motor.trace_revise(self.ball.center.x, self.ball.center.y, 25)
        if abs(motor.head_horizon - 2048) > ROTATE_MISTAKE:
            
            motor.body_trace_rotate(2048,ROTATE_MISTAKE)
        else:  # 1320是調球的距離150是誤差
            
            motor.body_trace_straight(KICK_DEGREE2 ,KICK_ERROR2)

    def imu_reset(self, fix, error_imu):         
    #讓身體imu回正，面向(回正)一開始方向
        self.yaw_fix = send.imu_value_Yaw - fix  # imu_value_yaw 當前yaw值
        if abs(self.yaw_fix) > error_imu:        # fix要修去的值 , error_imu 容忍範圍的誤差
            if self.yaw_fix > 0:                 # yaw_fix yaw要修的值           
                self.MoveContinuous(RIGHT_CORRECT[0], RIGHT_CORRECT[1], RIGHT_CORRECT[2], 500, 500, 1)
            elif self.yaw_fix < 0:
                self.MoveContinuous(LEFT_CORRECT[0], LEFT_CORRECT[1], LEFT_CORRECT[2], 500, 500, 1)
        elif abs(self.yaw_fix) < error_imu:
            self.step_jump = True
            motor.cnt = 3

if __name__ == '__main__':   # left : SMALL 7933 BIG : 7934
                             #RIGHT : small 7770 big :7911
    # i, x = 0, 0
    try:
        PK = PenaltyKick()
        motor = MotorMove()
        while not rospy.is_shutdown():
            if send.is_start :
                PK.main()
                
            else:
                PK.init()

    except rospy.ROSInterruptException:
        pass
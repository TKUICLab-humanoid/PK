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

CORRECT = [-1300, -400, 0]    
LEVEL_LEFT_CORRECT = [-1500, 3000, 1] 
LEVEL_RIGHT_CORRECT = [-1300, -3000, 0] 
LEFT_CORRECT = [-1500, -400, 3] 
RIGHT_CORRECT = [-1500, -400, -3] 

LOOK_BALL = 2600  #預先讓頭轉至大概角度
SLOWDOWN = [0,0]
ROTATE_MISTAKE = 50  # x , y , theta
KICK_DEGREE , KICK_ERROR = 2840, 20# 第一次小踢
KICK_DEGREE2 , KICK_ERROR2 = 2710, 20 #直接射門
KICK_DEGREE_MISTAKE = 50

class TargetLocation():
    def __init__(self):                                                                     #初始化參數
        self.color_mask_subject_blue_first= 0
        self.color_mask_subject_blue_second = 0
        self.color_mask_subject_orange = 0
        self.color_mask_subject_white = 0
#=====================================================================================================================
# obs最終的
        self.obs_x_left = 0
        self.obs_y_left = 0
        self.obs_size_left = 0
        self.obs_x_min_left = 0
        self.obs_y_min_left = 0
        self.obs_x_max_left = 0
        self.obs_y_max_left = 0
        
        self.obs_x_right = 0
        self.obs_y_right = 0
        self.obs_size_right = 0
        self.obs_x_min_right = 0
        self.obs_y_min_right = 0
        self.obs_x_max_right = 0
        self.obs_y_max_right = 0
#==========================================================================================================================暫存的 
#left obs暫存
        self.obs_x_left_list = [0]
        self.obs_y_left_list = [0]
        self.obs_width_left_list = [0]
        self.obs_height_left_list = [0]
        self.obs_x_min_left_list = [0]
        self.obs_y_min_left_list = [0]
        self.obs_x_max_left_list = [0]
        self.obs_y_max_left_list = [0]
        self.obs_size_left_list = [0]
#right obs暫存
        self.obs_x_right_list = [0]
        self.obs_y_right_list = [0]
        self.obs_width_right_list = [0]
        self.obs_height_right_list = [0]
        self.obs_x_min_right_list = [0]
        self.obs_y_min_right_list = [0]
        self.obs_x_max_right_list = [0]
        self.obs_y_max_right_list = [0]
        self.obs_size_right_list = [0]
#==========================================================================================================================
#line
        self.line_x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.line_y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.line_x_max = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.line_y_max = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.line_x_min = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.line_y_min = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.line_size = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#===========================================================================================================================ba
#ball
        self.ball_x = 0
        self.ball_y = 0
        self.ball_size = 0
        self.ball_x_min = 0
        self.ball_y_min = 0
        self.ball_x_max = 0
        self.ball_y_max = 0

        self.ball_x_list = [0]
        self.ball_y_list = [0]
        self.ball_size_list = [0]
        self.ball_x_min_list = [0]
        self.ball_y_min_list = [0]
        self.ball_x_max_list = [0]
        self.ball_y_max_list = [0]
#=========================================================================================================================
#只有一個obs
        self.obs_x = 0
        self.obs_y = 0
        self.obs_size = 0
        self.obs_x_min = 0
        self.obs_y_min = 0
        self.obs_x_max = 0
        self.obs_y_max = 0

        self.obs_x_list = [0]
        self.obs_y_list = [0]
        self.obs_size_list = [0]
        self.obs_x_min_list = [0]
        self.obs_y_min_list = [0]
        self.obs_x_max_list = [0]
        self.obs_y_max_list = [0]
        
    def one_obs_parameter(self):                                                            #當只有一個障礙物時
        self.color_mask_subject_blue_first = send.color_mask_subject_cnts[2]
        for j in range(self.color_mask_subject_blue_first):
            if send.color_mask_subject_XMin[2][j] > 50 and send.color_mask_subject_XMax[2][j] < 300:
                if send.color_mask_subject_size[2][j] > 600 and send.color_mask_subject_XMin[2][j] <250:
                    self.obs_x_list.append(send.color_mask_subject_X[2][j])
                    self.obs_y_list.append(send.color_mask_subject_Y[2][j])
                    self.obs_size_list.append(send.color_mask_subject_size[2][j])
                    self.obs_x_min_list.append(send.color_mask_subject_XMin[2][j])
                    self.obs_y_min_list.append(send.color_mask_subject_YMin[2][j])
                    self.obs_x_max_list.append(send.color_mask_subject_XMax[2][j])
                    self.obs_y_max_list.append(send.color_mask_subject_YMax[2][j])
                else :
                    self.obs_x_list.append(0)
                    self.obs_y_list.append(0)
                    self.obs_size_list.append(0)
                    self.obs_x_min_list.append(0)
                    self.obs_y_min_list.append(0)
                    self.obs_x_max_list.append(0)
                    self.obs_y_max_list.append(0)
#=======================================================================================================================
#list 取最大值
        self.obs_x = max(self.obs_x_list)
        self.obs_y = max(self.obs_y_list)
        self.obs_size = max(self.obs_size_list)
        self.obs_x_min = max(self.obs_x_min_list)
        self.obs_y_min = max(self.obs_y_min_list)
        self.obs_x_max = max(self.obs_x_max_list)
        self.obs_y_max = max(self.obs_y_max_list)

        self.obs_x_list = [0]
        self.obs_y_list = [0]
        self.obs_size_list = [0]
        self.obs_x_min_list = [0]
        self.obs_y_min_list = [0]
        self.obs_x_max_list = [0]
        self.obs_y_max_list = [0]

    def two_obs_parameter(self):                                                            #當有兩個障礙物時
        #left obs暫存
        self.color_mask_subject_blue_second = send.color_mask_subject_cnts[2]
        for j in range(self.color_mask_subject_blue_second):
            if send.color_mask_subject_size[2][j] > 500:
                if send.color_mask_subject_XMin[2][j] > 170 and send.color_mask_subject_XMax[2][j] <= 320:
                    self.obs_x_right_list.append(send.color_mask_subject_X[2][j])
                    self.obs_y_right_list.append(send.color_mask_subject_Y[2][j])
                    self.obs_size_right_list.append( send.color_mask_subject_size[2][j])
                    self.obs_x_min_right_list.append(send.color_mask_subject_XMin[2][j])
                    self.obs_y_min_right_list.append(send.color_mask_subject_YMin[2][j])
                    self.obs_x_max_right_list.append(send.color_mask_subject_XMax[2][j])
                    self.obs_y_max_right_list.append(send.color_mask_subject_YMax[2][j])
                    self.obs_width_right_list.append(send.color_mask_subject_Width[2][j])
                    self.obs_height_right_list.append( send.color_mask_subject_Height[2][j])
                
                elif send.color_mask_subject_XMax[2][j] < 160 and send.color_mask_subject_XMin[2][j] >= 0:
                    self.obs_x_left_list.append(send.color_mask_subject_X[2][j])
                    self.obs_y_left_list.append(send.color_mask_subject_Y[2][j])
                    self.obs_size_left_list.append(send.color_mask_subject_size[2][j])
                    self.obs_x_min_left_list.append(send.color_mask_subject_XMin[2][j])
                    self.obs_y_min_left_list.append(send.color_mask_subject_YMin[2][j])
                    self.obs_x_max_left_list.append( send.color_mask_subject_XMax[2][j])
                    self.obs_y_max_left_list.append(send.color_mask_subject_YMax[2][j])
                    self.obs_width_left_list.append(send.color_mask_subject_Width[2][j])
                    self.obs_height_left_list.append( send.color_mask_subject_Height[2][j])

            elif  send.color_mask_subject_size[2][j] < 500:
                if send.color_mask_subject_XMin[2][j] > 160 and send.color_mask_subject_XMax[2][j] <= 320:
                    self.obs_x_right_list.append(0)
                    self.obs_y_right_list.append(0)
                    self.obs_width_right_list.append(0)
                    self.obs_height_right_list.append(0)
                    self.obs_x_min_right_list.append(0)
                    self.obs_y_min_right_list.append(0)
                    self.obs_x_max_right_list.append(0)
                    self.obs_y_max_right_list.append(0)
                    self.obs_size_right_list.append(0)
                elif  send.color_mask_subject_XMax[2][j] < 160 and send.color_mask_subject_XMin[2][j] >= 0:
                    self.obs_x_left_list.append(0)
                    self.obs_y_left_list.append(0)
                    self.obs_width_left_list.append(0)
                    self.obs_height_left_list.append(0)
                    self.obs_x_min_left_list.append(0)
                    self.obs_y_min_left_list.append(0)
                    self.obs_x_max_left_list.append(0)
                    self.obs_y_max_left_list.append(0)
                    self.obs_size_left_list.append(0)

        
#====================================================================================================================================
# 把list的最大值輸進去
        self.obs_x_right = max(self.obs_x_right_list)
        self.obs_y_right = max(self.obs_y_right_list)
        self.obs_width_right = max(self.obs_width_right_list)
        self.obs_height_right = max(self.obs_height_right_list)
        self.obs_x_min_right = max(self.obs_size_right_list)
        
        self.obs_x_left = max(self.obs_x_left_list)
        self.obs_y_left = max(self.obs_y_left_list)
        self.obs_width_left = max(self.obs_width_left_list)
        self.obs_height_left = max(self.obs_height_left_list)
        self.obs_x_min_left = max(self.obs_x_min_left_list)
        self.obs_y_min_left = max(self.obs_y_min_left_list)
        self.obs_x_max_left = max( self.obs_x_max_left_list)
        self.obs_y_max_left = max(self.obs_y_max_left_list)
        self.obs_size_left = max(self.obs_size_left_list)
#left obs暫存
        self.obs_x_left_list = [0]
        self.obs_y_left_list = [0]
        self.obs_width_left_list = [0]
        self.obs_height_left_list = [0]
        self.obs_x_min_left_list = [0]
        self.obs_y_min_left_list = [0]
        self.obs_x_max_left_list = [0]
        self.obs_y_max_left_list = [0]
        self.obs_size_left_list = [0]

#right obs暫存
        self.obs_x_right_list = [0]
        self.obs_y_right_list = [0]
        self.obs_width_right_list = [0]
        self.obs_height_right_list = [0]
        self.obs_x_min_right_list = [0]
        self.obs_y_min_right_list = [0]
        self.obs_x_max_right_list = [0]
        self.obs_y_max_right_list = [0]
        self.obs_size_right_list = [0]
        # obs若移動過快會無法歸零！！！！！！！！！！

    def ball_parameter(self):                                                               #判斷球
        self.color_mask_subject_orange = send.color_mask_subject_cnts[0]
        for j in range(self.color_mask_subject_orange):
            if send.color_mask_subject_size[0][j] > 600 :
                self.ball_x_list.append(send.color_mask_subject_X[0][j])
                self.ball_y_list.append(send.color_mask_subject_Y[0][j])
                self.ball_size_list.append(send.color_mask_subject_size[0][j])
                self.ball_x_min_list.append(send.color_mask_subject_XMin[0][j])
                self.ball_y_min_list.append(send.color_mask_subject_YMin[0][j])
                self.ball_x_max_list.append(send.color_mask_subject_XMax[0][j])
                self.ball_y_max_list.append(send.color_mask_subject_YMax[0][j])
            else :
                self.ball_x_list.append(0)
                self.ball_y_list.append(0)
                self.ball_size_list.append(0)
                self.ball_x_min_list.append(0)
                self.ball_y_min_list.append(0)
                self.ball_x_max_list.append(0)
                self.ball_y_max_list.append(0)
#=======================================================================================================================
#list 取最大值
        self.ball_x = max(self.ball_x_list)
        self.ball_y = max(self.ball_y_list)
        self.ball_size = max(self.ball_size_list)
        self.ball_x_min = max(self.ball_x_min_list)
        self.ball_y_min = max(self.ball_y_min_list)
        self.ball_x_max = max(self.ball_x_max_list)
        self.ball_y_max =max(self.ball_y_max_list)

        self.ball_x_list = [0]
        self.ball_y_list = [0]
        self.ball_size_list = [0]
        self.ball_x_min_list = [0]
        self.ball_y_min_list = [0]
        self.ball_x_max_list = [0]
        self.ball_y_max_list = [0]
                    
    def line_parameter(self):                                                               #判斷線(目前還沒用到)
        self.color_mask_subject_white = send.color_mask_subject_cnts[6]
        for j in range(self.color_mask_subject_white):
            if send.color_mask_subject_size[6][j] > 1000:
                self.line_x[j] = send.color_mask_subject_X[6][j]
                self.line_y[j] = send.color_mask_subject_Y[6][j]
                self.line_size[j] = send.color_mask_subject_size[6][j]
                self.line_x_min[j] = send.color_mask_subject_XMin[6][j]
                self.line_y_min[j] = send.color_mask_subject_YMin[6][j]
                self.line_x_max[j] = send.color_mask_subject_XMax[6][j]
                self.line_y_max[j] = send.color_mask_subject_YMax[6][j]
            else:
                self.line_x[j] = 0
                self.line_y[j] = 0
                self.line_size[j] = 0
                self.line_x_min[j] = 0
                self.line_y_min[j] = 0
                self.line_x_max[j] = 0
                self.line_y_max[j] = 0

class MotorMove():

    def __init__(self):
        self.head_horizon   = 2048     # 頭部水平刻度
        self.head_vertical  = 2048    # 頭部垂直刻度
        self.waist_position = 2048   # 腰當下的刻度
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
        self.MoveY = 0              # 轉腰修正頭的高度
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
                
    def view_move(self, right_place, left_place, down_place, up_place, speed, delay):       #搜尋目標物，掃描畫面
        if self.head_horizon_flag == 1:
            if self.head_horizon > left_place:
                self.move_head(1, self.head_horizon, 880, 880, speed)
                self.head_horizon = self.head_horizon - speed
                time.sleep(delay)
            else:
                self.move_head(2, up_place, 880, 880, 100)
                self.head_horizon_flag = 0
        else:
            if self.head_horizon < right_place:
                self.move_head(1, self.head_horizon, 880, 880, speed)
                self.head_horizon = self.head_horizon + speed
                time.sleep(delay)
            else:
                self.move_head(2, down_place, 880, 880, 100)
                self.head_horizon_flag = 1

    def trace_revise(self, x_target, y_target, speed):                                      #追蹤目標物
        self.x_differ = x_target - 160
        self.y_differ = y_target - 120
        self.x_degree = self.x_differ * (64.5 / 320)
        self.y_degree = self.y_differ * (-40 / 240) # 70.42 43.3 要重新測
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
                self.MoveContinuous(3000 + CORRECT[0], 0 + CORRECT[1], 0 + CORRECT[2], 500 , 500, 1)
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

    def imu_yaw_reset(self, fix, error_imu):                                                #讓身體imu回正，面向(回正)一開始方向
        self.yaw_fix = send.imu_value_Yaw - fix  # imu_value_yaw 當前yaw值
        if abs(self.yaw_fix) > error_imu:        # fix要修去的值 , error_imu 容忍範圍的誤差
            if self.yaw_fix > 0:                 # yaw_fix yaw要修的值           
                self.MoveContinuous(RIGHT_CORRECT[0], RIGHT_CORRECT[1], RIGHT_CORRECT[2], 500, 500, 1)
            elif self.yaw_fix < 0:
                self.MoveContinuous(LEFT_CORRECT[0], LEFT_CORRECT[1], LEFT_CORRECT[2], 500, 500, 1)
        elif abs(self.yaw_fix) < error_imu:
            self.step_jump = True
            motor.cnt = 3

    def bodyauto_close(self, next_state):                                                   #reset機器人狀態，控制步態開關
        if self.now_state == next_state:
            pass
        elif self.now_state != next_state:
            send.sendBodyAuto(0,0,0,0,1,0)
            time.sleep(1)
            self.now_state = next_state

class StepState():
    def __init__(self):
        self.step_now = "begin"

    def step_begin(self):
        motor.move_head(2, 2048, 880, 880, 50)
        send.sendSensorReset()
        time.sleep(0.5)
        self.step_now = "test"

    def step_test(self):
        self.step_now = "search_ball"

    def step_search_ball(self,right,left,up,down):
        target.ball_parameter()
        if target.ball_size < 800:  # 500要測
            motor.view_move(right, left, up, down, 70, 0.05)
            time.sleep(0.05)
            target.ball_parameter()
        elif target.ball_size > 800:
            time.sleep(0.1)
            motor.step_jump = True

    def step_start_trace_ball(self,left_or_right):
        target.ball_parameter()
        if abs(target.ball_x - 160) > 8 or abs(target.ball_y - 120) > 6:
            target.ball_parameter()
            motor.trace_revise(target.ball_x, target.ball_y, 50)
            time.sleep(0.05)
        else:
            rospy.loginfo(f"motor.head_horizon{motor.head_horizon}")
            if motor.head_horizon >= left_or_right:
                time.sleep(0.05)
                motor.move_head(1, 2048, 880, 880, 50)
                motor.move_head(2, 2770, 880, 880, 50)
                time.sleep(0.05)
                motor.bodyauto_close(1)
                motor.MoveContinuous(CORRECT[0], CORRECT[1], CORRECT[2], 500, 500, 1)  # 原地踏步
                time.sleep(0.1)
                rospy.loginfo("find left left left")
                self.step_now  = "walk_ball_left"
            else:
                time.sleep(0.05)
                motor.move_head(1, 2048, 880, 880, 50)
                motor.move_head(2, 2770, 880, 880, 50)
                time.sleep(0.05)
                motor.bodyauto_close(1)
                motor.MoveContinuous(CORRECT[0], CORRECT[1], CORRECT[2], 500, 500, 1)  # 原地踏步
                time.sleep(0.1)
                target.ball_parameter()
                rospy.loginfo("find right right right")
                self.step_now = "walk_ball_right"

    def step_walk_ball_left(self,bound):
        send.drawImageFunction(8, 0, bound, bound, 0, 320, 255, 255, 255)  # 對球中心線
        target.ball_parameter()
        if motor.cnt > 0:
            if target.ball_x_min < bound:
                rospy.loginfo("go left")
                motor.MoveContinuous(LEVEL_LEFT_CORRECT[0] - SLOWDOWN[0], LEVEL_LEFT_CORRECT[1] - SLOWDOWN[1],
                                     LEVEL_LEFT_CORRECT[2], 500, 500, 1)
                motor.cnt= 3  # 要讓球最小值離開過三次界線才跳出 預防步態不穩
            else:
                motor.cnt -= 1
        else:
            motor.cnt = 2
            time.sleep(0.2)
            target.ball_parameter()
            self.step_now = "trace_ball"

    def step_walk_ball_right(self,bound):
        send.drawImageFunction(8, 0, bound, bound, 0, 320, 255, 255, 255)  # 對球中心線
        target.ball_parameter()
        rospy.loginfo("aa%d,%d",motor.cnt,target.ball_x_min)
        if motor.cnt > 0:
            if target.ball_x_min > bound or target.ball_x_min == 0:
                motor.MoveContinuous(LEVEL_RIGHT_CORRECT[0] - SLOWDOWN[0], LEVEL_RIGHT_CORRECT[1] - SLOWDOWN[1], LEVEL_RIGHT_CORRECT[2], 500, 500, 1)
                motor.cnt = 3  # 要讓球最小值離開過三次界線才跳出 預防步態不穩
            else:
                motor.cnt -= 1
        elif motor.cnt <= 0:
            rospy.loginfo("bb")
            time.sleep(0.2)
            motor.step_jump = True

    def step_kick_trace_ball(self):
        target.ball_parameter()
        motor.trace_revise(target.ball_x, target.ball_y, 50)  # 追蹤球
        if abs(motor.head_vertical - KICK_DEGREE) <= KICK_ERROR:  # 跟球距離洽當就踢   20 要再側
            rospy.loginfo("small kick")  # 小踢
            motor.cnt = motor.cnt - 1
        elif abs(send.imu_value_Yaw) > 5:
            motor.imu_yaw_reset(0, 5)
            rospy.loginfo("reset ")
            motor.cnt = 2
        elif abs(motor.head_vertical - KICK_DEGREE) > KICK_ERROR:  # 如果太遠
            if motor.head_vertical - KICK_DEGREE < KICK_ERROR:
                motor.MoveContinuous(CORRECT[0] + 2200, CORRECT[1], CORRECT[2], 500, 500, 1)
                rospy.loginfo("前進")
                motor.cnt = 2
            elif motor.head_vertical - KICK_DEGREE > KICK_ERROR:
                motor.MoveContinuous(CORRECT[0] - 300, CORRECT[1], CORRECT[2], 500, 500, 1)
                rospy.loginfo("後退")
                motor.cnt = 2
        if motor.cnt == 0:
            motor.bodyauto_close(0)
            time.sleep(4)
            send.sendBodySector(7770)
            time.sleep(2)
            motor.move_head(2, LOOK_BALL, 880, 880, 50)
            motor.move_head(1, 2048, 880, 880, 50)
            time.sleep(6)
            motor.bodyauto_close(1)
            motor.MoveContinuous(CORRECT[0], CORRECT[1], CORRECT[2], 500, 500, 1) 
            self.step_now = "imu_reset_obs"
            motor.cnt = 5

    def step_score_ahead(self,score_vertical):
        target.ball_parameter()
        rospy.loginfo(f"ver{motor.head_vertical}")  # 小踢
        motor.trace_revise(target.ball_x, target.ball_y, 50)  # 追蹤球
        if abs(motor.head_vertical - score_vertical) <= 30:  # 跟球距離洽當就踢   20 要再側
            rospy.loginfo("small kick")  # 小踢
            motor.cnt = motor.cnt - 1
        elif abs(send.imu_value_Yaw) > 5:
            motor.imu_yaw_reset(0, 5)
            rospy.loginfo("reset ")
            motor.cnt = 2
        elif abs(motor.head_vertical - score_vertical) > 30:  # 如果太遠
            if motor.head_vertical - score_vertical < score_vertical:
                motor.MoveContinuous(CORRECT[0] + 2200, CORRECT[1], CORRECT[2], 500, 500, 1)
                rospy.loginfo("前進")
                motor.cnt = 2
            elif motor.head_vertical - score_vertical > 30:
                motor.MoveContinuous(CORRECT[0] - 300, CORRECT[1], CORRECT[2], 500, 500, 1)
                rospy.loginfo("後退")
                motor.cnt = 2
        if motor.cnt == 0:
            motor.step_jump = True

    def step_imu_reset_obs(self):
        motor.imu_yaw_reset(0, 5)
        

    def step_avoid_obs(self,avoidobs):
        send.drawImageFunction(10, 0, avoidobs, avoidobs, 0, 320, 255, 0, 255)  # 對球中心線
        target.one_obs_parameter()
        rospy.loginfo("aaaa%d,%d",target.obs_x_max,motor.cnt)
        if motor.cnt > 0:
            if target.obs_x_max > avoidobs:
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
            self.step_now = "kick_search_ball"

    def kick_search_to_ball(self):
        if motor.step_jump:
            self.step_now = "kick_trace_ball"
            
        elif self.step_now == "kick_trace_ball": #持續盯著球
            target.ball_parameter()
            if abs(target.ball_x - 160) > 8 or abs(target.ball_y - 120) > 6:
                target.ball_parameter()
                motor.trace_revise(target.ball_x, target.ball_y, 50)
            else:
                self.step_now = 'obs.walk_to_ball'

    def obs_walk_to_ball(self):
        target.ball_parameter()
        target.one_obs_parameter()
        motor.trace_revise(target.ball_x, target.ball_y, 50)
        send.drawImageFunction(3, 1, target.ball_x_min, target.ball_x_max, target.ball_y_min,target.ball_y_max, 255, 0, 255) #對球畫框定位球位置
        send.drawImageFunction(5, 0, 100, 100, 0, 240, 100, 255, 0)  # 左(green) 畫不出線不知道就重開！！
        send.drawImageFunction(6, 0, 220, 220, 0, 240, 100, 255, 0)  # 右(green)
        if motor.head_horizon > 2650:
            ROTATE_MISTAKE = 110
            if motor.step_jump :
                motor.step_jump = False
                motor.move_head(2, 2500, 880, 880, 30)#頭往下
                motor.bodyauto_close(0)
                self.step_now = "obs.obs_check"

            elif target.obs_size_left > 0 or target.obs_size_right > 0:
                if target.obs_x_max_left > 100 :
                    rospy.loginfo("turn right")
                    motor.MoveContinuous(RIGHT_CORRECT[0]+2000, RIGHT_CORRECT[1], RIGHT_CORRECT[2], 500, 500, 1)
                    
                elif target.obs_x_min_right < 220 and target.obs_x_max_right != 0:
                    rospy.loginfo("turn light")
                    motor.MoveContinuous(LEFT_CORRECT[0]+2000, LEFT_CORRECT[1], LEFT_CORRECT[2], 500, 500, 1)
                else:
                    if abs(motor.head_horizon - 2048) > ROTATE_MISTAKE:#純轉向球
                        motor.body_trace_rotate(2048,ROTATE_MISTAKE)
                        rospy.loginfo(f"motor.head_vertical========={motor.head_vertical}" )
                    elif abs(motor.head_vertical - KICK_DEGREE2) > KICK_ERROR2: 
                        target.ball_parameter()
                        motor.body_trace_straight(KICK_DEGREE2 ,KICK_ERROR2)
                    elif abs(motor.head_vertical - KICK_DEGREE2) < KICK_ERROR2:
                        motor.step_jump = True

            elif(target.obs_size_left <= 0 and target.obs_size_right <= 0):
                if abs(motor.head_horizon - 2048) > ROTATE_MISTAKE:#純轉向球
                    motor.body_trace_rotate(2048,ROTATE_MISTAKE)
                    rospy.loginfo(f"motor.head_vertical========={motor.head_vertical}")
                elif abs(motor.head_vertical - KICK_DEGREE2) > KICK_ERROR2: 
                    target.ball_parameter()
                    motor.body_trace_straight(KICK_DEGREE2 ,KICK_ERROR2)
                elif abs(motor.head_vertical - KICK_DEGREE2) < KICK_ERROR2:
                    motor.step_jump = True

    def obs_check(self):
        target.one_obs_parameter()
        motor.move_head(2, 2800, 880, 880, 30) 
        self.step_now = 'ball.search_to_ball'

    def search_to_ball(self):
        if target.ball_size < 1000:
            motor.view_move(2648, 1498, 2750, 2550, 70, 0.05) #避障後搜球確認球位置
            time.sleep(0.05)
            target.ball_parameter()
        elif target.ball_size > 1000:
            self.step_now = 'ball.watch_ball'

    def watch_ball(self):
        if abs(target.ball_x - 160) > 8 or abs(target.ball_y - 120) > 6:
            target.ball_parameter()
            motor.trace_revise(target.ball_x, target.ball_y, 25)
            time.sleep(0.05)
        else:
            motor.bodyauto_close(1)
            self.step_now = 'ball.walk_to_ball'

    def walk_to_ball(self):
        motor.trace_revise(target.ball_x, target.ball_y, 25)
        if abs(motor.head_horizon - 2048) > ROTATE_MISTAKE:
            target.ball_parameter()
            motor.body_trace_rotate(2048,ROTATE_MISTAKE)
        else:  # 1320是調球的距離150是誤差
            target.ball_parameter()
            motor.body_trace_straight(KICK_DEGREE2 ,KICK_ERROR2)

    def imu_reset(self):
        motor.imu_yaw_reset(0, 5)

    def xx(self):
        target.ball_parameter()
        motor.trace_revise(target.ball_x, target.ball_y, 50)#追蹤球
        if abs(motor.head_vertical - KICK_DEGREE) <= KICK_ERROR:#跟球距離洽當就踢   20 要再側
            motor.cnt = motor.cnt -1
        elif abs(send.imu_value_Yaw) > 5 :
                motor.imu_yaw_reset(0,5)
                motor.cnt = 2
        elif abs(motor.head_vertical - KICK_DEGREE) > KICK_ERROR:#如果太遠
            if motor.head_vertical - KICK_DEGREE < KICK_ERROR :
                motor.MoveContinuous(CORRECT[0]+2200,CORRECT[1], CORRECT[2], 500, 500, 1)
                motor.cnt = 2
            elif motor.head_vertical - KICK_DEGREE > KICK_ERROR :
                motor.MoveContinuous(CORRECT[0]-300,CORRECT[1], CORRECT[2], 500, 500, 1)
                motor.cnt = 2
        if motor.cnt ==  0: 
            motor.bodyauto_close(0)
            send.sendBodySector(7771)
            
    def kick_search_to_ball(self):
        if target.ball_size < 1000:
            motor.view_move(2548, 1548, 2750,2550, 70, 0.05)
            target.ball_parameter()
        elif target.ball_size > 1000:
            self.step_now = 'kick.trace_ball'

    def kick_trace_ball(self):
        if abs(target.ball_x - 160) > 8 or abs(target.ball_y - 120) > 6:
            target.ball_parameter()
            motor.trace_revise(target.ball_x, target.ball_y, 25)
        else:
            self.step_now = 'kick.walk_to_ball'

    def kick_walk_to_ball(self):
        target.ball_parameter()
        motor.trace_revise(target.ball_x, target.ball_y, 25)
        motor.body_trace_level(2048,2850,50,70)
        if motor.step_jump == True:
            self.step_now = 'kick.ready_kick_ball'

    def init(self):
        # if step != 'begin' and (send.DIOValue == 8 or send.DIOValue == 9 or send.DIOValue == 10 or send.DIOValue == 12 or send.DIOValue == 13
        #or send.DIOValue == 11 or send.DIOValue == 15 or send.DIOValue == 24 or send.DIOValue == 27 or send.DIOValue == 31):
        if self.step_now != 'begin' and send.is_start == False:  # gazebo test
            motor.bodyauto_close(0)  
            target.__init__()
            motor.__init__()
            self.__init__()
            send.sendHeadMotor(1, 2048, 30)  # reset head
            send.sendHeadMotor(2, 2048, 30)
            time.sleep(1)
            send.sendBodySector(29)  # reset body

    def main(self):
        time.sleep(0.05)
        send.drawImageFunction(1, 0, 160, 160, 110, 130, 255, 255, 255)
        send.drawImageFunction(2, 0, 150, 170, 120, 120, 255, 255, 255)  # 對球中心線
        target.ball_parameter()
        target.line_parameter()

#=============================================================================================================  開始
#---------------open_ball----------------------open_ball-------------------open_ball---------------open_ball--------------open_ball
        if self.step_now == "begin":
            self.step_begin()

        elif self.step_now == "test":
            self.step_test()

        elif self.step_now == "search_ball":#小踢完後收尋球確認球的位置
            # motor.MoveContinuous(CORRECT[0], CORRECT[1], CORRECT[2], 500, 500, 1)  # 原地踏步
            self.step_search_ball(2448, 1648, 2550, 2250)
            if motor.step_jump:
                self.step_now = "start_trace_ball"
                motor.step_jump = False

        elif self.step_now == 'start_trace_ball':  # 持續盯著球
            self.step_start_trace_ball(1800)

        elif self.step_now == "walk_ball_left":  #開球是往左平移
            self.step_walk_ball_left(175)
            if motor.step_jump:
                self.step_now = "trace_ball"
                motor.step_jump = False
                motor.cnt = 3

        elif self.step_now == "walk_ball_right":  #開球是往左平移
            self.step_walk_ball_right(200)
            if motor.step_jump:
                self.step_now = "trace_ball"
                motor.step_jump = False
                motor.cnt = 3

        elif self.step_now == 'trace_ball':
            self.step_kick_trace_ball()

        elif self.step_now == "imu_reset_obs":
            self.step_imu_reset_obs()
            if motor.step_jump == True:
                self.step_now = "avoid_obs"
                time.sleep(2)
                motor.step_jump = False

        elif self.step_now == "avoid_obs":
            self.step_avoid_obs(150)

        elif self.step_now == "kick_search_ball":#小踢完後收尋球確認球的位置
            self.step_search_ball(2548, 1548, 2750, 2550)
            if motor.step_jump:
                self.step_now = "walk_to_ball"
                motor.step_jump = False
            
        elif self.step_now == 'obs.walk_to_ball':
            self.walk_to_ball()
                
        elif self.step_now == 'obs.obs_check':
            self.obs_check()
            
        elif self.step_now == "ball.search_to_ball":
            self.search_to_ball()
            
        elif self.step_now == "ball.watch_ball":
            self.watch_ball()
            
        elif self.step_now =="walk_to_ball":
            self.walk_to_ball()
            if motor.step_jump:
                self.step_now = "kick.imu_reset"
                motor.step_jump = False

        elif self.step_now == "kick.imu_reset":
            self.step_imu_reset_obs()
            if motor.step_jump:
                self.step_now = "kick.move_to_ball"
                motor.move_head(2,2700, 880, 880, 50)
                motor.move_head(1,2048, 880, 880, 50)
                time.sleep(0.1)
                motor.MoveContinuous(CORRECT[0], CORRECT[1], CORRECT[2], 500, 500, 1)
                target.ball_parameter()
                motor.step_jump = False
            
        elif self.step_now == "kick.move_to_ball":
            self.step_walk_ball_right(150)
            time.sleep(0.02)
            if motor.step_jump :
                motor.cnt = 3 
                time.sleep(0.02)
                self.step_now = "kick.goahead_to_ball"
                motor.step_jump = False

        elif self.step_now == "kick.goahead_to_ball":
            self.step_score_ahead(2840)
            if motor.step_jump :
                motor.bodyauto_close(0)
                time.sleep(1)
                send.sendBodySector(7911)
                self.step_now = "finish"
                motor.step_jump = False 
            
                
        
        # elif step.step_now == 'kick.search_to_ball':
        #     step.search_to_ball(2548, 1548, 2750, 2550)

        # elif step.step_now == 'kick.trace_ball':
        #     step.kick_trace_ball()

        # elif step.step_now == 'kick.walk_to_ball':
        #     step.kick_walk_to_ball()
                    
                        
if __name__ == '__main__':
    # i, x = 0, 0
    try:
        target = TargetLocation()
        motor = MotorMove()
        step = StepState()
        while not rospy.is_shutdown():
            if send.is_start:
                step.main()

            else:
                step.init()


    except rospy.ROSInterruptException:
        pass
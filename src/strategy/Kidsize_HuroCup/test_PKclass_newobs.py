#!/usr/bin/env python
# coding=utf-8
from cmath import sqrt
from re import T
import time
from traceback import print_tb

import numpy as np
import rospy
from Python_API import Sendmessage

send = Sendmessage()


class target_location():
    def __init__(self):
        self.color_mask_subject_blue = 0
        self.color_mask_subject_blue2 = 0
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
        self.obs2_x = 0
        self.obs2_y = 0
        self.obs2_size = 0
        self.obs2_x_min = 0
        self.obs2_y_min = 0
        self.obs2_x_max = 0
        self.obs2_y_max = 0

        self.obs2_x_list = [0]
        self.obs2_y_list = [0]
        self.obs2_size_list = [0]
        self.obs2_x_min_list = [0]
        self.obs2_y_min_list = [0]
        self.obs2_x_max_list = [0]
        self.obs2_y_max_list = [0]

    def obs_parameter(self):
        #left obs暫存
        self.color_mask_subject_blue = send.color_mask_subject_cnts[2]
        for j in range(self.color_mask_subject_blue):
            if send.color_mask_subject_size[2][j] > 500:
                if send.color_mask_subject_XMin[2][j] > 160 and send.color_mask_subject_XMax[2][j] <= 320:
                    self.obs_x_right_list.append(send.color_mask_subject_X[2][j])
                    self.obs_y_right_list.append(send.color_mask_subject_Y[2][j])
                    self.obs_size_right_list.append( send.color_mask_subject_size[2][j])
                    self.obs_x_min_right_list.append(send.color_mask_subject_XMin[2][j])
                    self.obs_y_min_right_list.append(send.color_mask_subject_YMin[2][j])
                    self.obs_x_max_right_list.append(send.color_mask_subject_XMax[2][j])
                    self.obs_y_max_right_list.append(send.color_mask_subject_YMax[2][j])
                    self.obs_width_right_list.append(send.color_mask_subject_Width[2][j])
                    self.obs_height_right_list.append( send.color_mask_subject_Height[2][j])
                


                elif send.color_mask_subject_XMax[2][j] < 160 and send.color_mask_subject_XMin[2][j] >=0:
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
        self.obs_x_min_right = max(self.obs_x_min_right_list)
        self.obs_y_min_right = max(self.obs_y_min_right_list)
        self.obs_x_max_right = max( self.obs_x_max_right_list)
        self.obs_y_max_right = max(self.obs_y_max_right_list)
        self.obs_size_right = max(self.obs_size_right_list)
        
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

    def obs2_parameter(self):
        self.color_mask_subject_blue2 = send.color_mask_subject_cnts[2]
        for j in range(self.color_mask_subject_blue2):
            if send.color_mask_subject_XMin[2][j] > 50 and send.color_mask_subject_XMax[2][j] < 300:
                if send.color_mask_subject_size[2][j] > 600 and send.color_mask_subject_XMin[2][j] <250:
                    self.obs2_x_list.append(send.color_mask_subject_X[2][j])
                    self.obs2_y_list.append(send.color_mask_subject_Y[2][j])
                    self.obs2_size_list.append(send.color_mask_subject_size[2][j])
                    self.obs2_x_min_list.append(send.color_mask_subject_XMin[2][j])
                    self.obs2_y_min_list.append(send.color_mask_subject_YMin[2][j])
                    self.obs2_x_max_list.append(send.color_mask_subject_XMax[2][j])
                    self.obs2_y_max_list.append(send.color_mask_subject_YMax[2][j])
                else :
                    self.obs2_x_list.append(0)
                    self.obs2_y_list.append(0)
                    self.obs2_size_list.append(0)
                    self.obs2_x_min_list.append(0)
                    self.obs2_y_min_list.append(0)
                    self.obs2_x_max_list.append(0)
                    self.obs2_y_max_list.append(0)

#=======================================================================================================================
#list 取最大值
        
        self.obs2_x = max(self.obs2_x_list)
        self.obs2_y = max(self.obs2_y_list)
        self.obs2_size = max(self.obs2_size_list)
        self.obs2_x_min = max(self.obs2_x_min_list)
        self.obs2_y_min = max(self.obs2_y_min_list)
        self.obs2_x_max = max(self.obs2_x_max_list)
        self.obs2_y_max = max(self.obs2_y_max_list)

        self.obs2_x_list = [0]
        self.obs2_y_list = [0]
        self.obs2_size_list = [0]
        self.obs2_x_min_list = [0]
        self.obs2_y_min_list = [0]
        self.obs2_x_max_list = [0]
        self.obs2_y_max_list = [0]



#==========================================================================================================================================


    def ball_parameter(self):
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

#===============================================================================================================================

    def line_parameter(self):
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


class motor_move():
    # motor = motor_move()
    target = target_location()
    motor = target_location()

    def __init__(self):
        self.head_horizon = 2048  # 頭部水平刻度
        self.head_vertical = 2048  # 頭部垂直刻度
        self.waist_position = 2048  # 腰當下的刻度
        self.head_horizon_flag = 0  # 找目標物的旗標
        self.x_differ = 0  # 目標與中心x差距
        self.y_differ = 0  # 目標與中心y差距
        self.x_degree = 0  # 目標與中心x角度
        self.y_degree = 0
        self.body_level = 0  # 目標與中心y角度
        self.body_rotate = 0  # 身體要旋轉多少？？？
        self.body_straight = 0
        self.found = False
        self.catch = False
        self.now_state = 0
        self.Move_waist = 0  # 修腰
        self.MoveY = 0  # 轉腰修正頭的高度
        self.NowX = 0  # 現在要移動的x量
        self.NowY = 0  # 現在要移動的x量
        self.NowTheta = 0     
        send.Web = True
        self.step_jump = False

    # 開球： 平移兩部後朝向球 小踢

    def move_head(self, ID, Position, max_head_horizon_size, max_head_vertical_size, Speed):
        # send.sendHeadMotor(ID,Position,Speed)
        
        # if ID == 1 :
        #     self.head_horizon =  Position
            

        # elif ID == 2 :
        #     self.head_vertical = Position

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
            # self.head_vertical = Position
            # send.sendHeadMotor(ID,Position,Speed)
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
                
    def view_move(self, right_place, left_place, down_place, up_place, speed, delay):

        if self.head_horizon_flag == 1:
            if self.head_horizon > left_place:
                self.move_head(1, self.head_horizon, 880, 880, speed)
                self.head_horizon = self.head_horizon - speed
                print("1")
                time.sleep(delay)
            else:
                self.move_head(2, up_place, 880, 880, 100)  #極限沒屁用要看看
                print("2")
                self.head_horizon_flag = 0
        else:
            if self.head_horizon < right_place:
                self.move_head(1, self.head_horizon, 880, 880, speed)
                self.head_horizon = self.head_horizon + speed
                print("3")
                time.sleep(delay)
            else:
                self.move_head(2, down_place, 880, 880, 100)
                print("4")
                self.head_horizon_flag = 1

    def trace_revise(self, x_target, y_target, speed):
        # if abs(x_target) > 0 and abs(y_target) > 0:
        self.x_differ = x_target - 160
        self.y_differ = y_target - 120
        self.x_degree = self.x_differ * (64.5 / 320)
        self.y_degree = self.y_differ * (-40 / 240)
        # 70.42 43.3 要重新測
        self.move_head(1, self.head_horizon - round(self.x_degree * 4096 / 360 * 0.15), 880, 880, speed)
        self.move_head(2, self.head_vertical - round(self.y_degree * 4096 / 360 * 0.15), 800, 800, speed)
        time.sleep(0.05)

    def body_trace_rotate(self,spot_degree,error): #我要到的角度 , 誤差 
        self.body_rotate = self.head_horizon - spot_degree
        if self.body_rotate > error:
            motor.MoveContinuous(left_correct[0], left_correct[1], left_correct[2], 500, 500, 1)
            print("go left = ", self.body_rotate)
            time.sleep(0.05)
        elif self.body_rotate < -error:
            motor.MoveContinuous(right_correct[0], right_correct[1], right_correct[2], 500, 500, 1)  # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            print("go right = ", self.body_rotate)
            time.sleep(0.05)

    def body_trace_level(self, degree_level, degree_straight, error_level,error_straight):  # 要的水平角度  要的直走角度 容忍的水平角度誤差 容忍的直走角度
        self.body_level = self.head_horizon - degree_level
        self.body_straight = self.head_vertical - degree_straight
        print(self.body_level,self.body_straight)
        if abs(self.body_level) > error_level:
            if self.body_level > 0:  # 左正右負
                print("go left level ")
                time.sleep(0.05)
            elif self.body_level < 0:
                print("go right level ")
                time.sleep(0.05)
        elif abs(self.body_straight) > error_straight:
            if self.body_straight > 0:
                print("go back")
                time.sleep(0.05)
            elif self.body_straight < 0:
                print("go ahead ")
                time.sleep(0.05)
        else:
            motor.step_jump = True


    def body_trace_straight(self, spot_degree, error): #目標的點  ,誤差

            if (self.head_vertical - spot_degree) < -error:
                motor.MoveContinuous(3000 + correct[0], 0 + correct[1], 0 + correct[2], 500 , 500,1)  # !!!!!!!!!!!!!!!!!!!!!!
                print("go ahead  ", self.head_vertical)
                time.sleep(0.05)

            elif (self.head_vertical - spot_degree) > error:
                motor.MoveContinuous(-900 + correct[0], 0 + correct[1], 0 + correct[2], 500 , 500,1)
                print("go back = ", self.head_vertical)
                time.sleep(0.05)
            
            elif abs(self.head_vertical - spot_degree) <= error:
                # send.sendBodyAuto(0,0,0,0,1,0)
                self.step_jump = True
                print("hi")
                time.sleep(1)



    ####################################### basket degree version #######################################

    def body_trace_basket_straight_2(self, degree, basket_error):

        if self.head_vertical - degree > basket_error and self.head_vertical > 1980:
            motor.MoveContinuous(2600 + correct[0], 0 + correct[1], 0 + correct[2], 100, 100, 1)  # !!!!!!!!!!!!!!!
            print("--------------------go ahead bbbb to basket---------------------  ", self.head_vertical)
            time.sleep(0.05)

        elif self.head_vertical - degree > basket_error and self.head_vertical < 1980:
            motor.MoveContinuous(1000 + correct[0], 0 + correct[1], 0 + correct[2], 100, 100, 1)  # !!!!!!!!!!!!!!!
            print("--------------------go ahead sss to basket---------------------  ", self.head_vertical)
            time.sleep(0.05)
        elif self.head_vertical - degree < basket_error and abs(self.head_vertical - degree) > basket_error:
            motor.MoveContinuous(-800 + correct[0], 0 + correct[1], 0 + correct[2], 100, 100, 1)  # !!!!!!!!!!!!!!

            print("--------------------go back from basket-------------------- ", self.head_vertical)
            time.sleep(0.05)
        elif abs(self.head_vertical - degree) < basket_error:
            print("--------------------stop at the basket----------------------", self.head_vertical - 2048)
            # send.sendBodyAuto(0,0,0,0,1,0)
            # motor.bodyauto_close(0)


            time.sleep(1)
            motor.move_head(1, 1798, 880, 880, 30)
            time.sleep(0.8)
            send.sendBodySector(3)  # 準備上籃之動作一（舉手...）111111111111111111111111111111111111111111111111111111111`
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(2.5)
            self.found = True
            self.catch = True

    #####################################################################################################

    def MoveContinuous(self, ExpX, ExpY, ExpTheta, AddX, AddY, AddTheta):
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

        # print("NowX, NowY, NowTheta = ", self.NowX, ", ", self.NowY, ", ", self.NowTheta)
        send.sendContinuousValue(self.NowX, self.NowY, 0, self.NowTheta, 0)

    def test_distance(self):

        target.basket_parameter()
        # if target.basket_size >= 300 and abs(motor.head_horizon-2048) < 600:

        if abs(target.basket_x - 160) > 5 or abs(target.basket_y - 120) > 4:
            motor.trace_revise(target.basket_x, target.basket_y, 25)
            target.basket_parameter()

            time.sleep(0.05)

        else:
            print("Basket Y = ", target.basket_size)

    def basket_distance(self, six, nine):
        print("target.basket_y - 120", target.basket_y - 120)
        self.move_head(1, 2048, 880, 880, 50)
        target.basket_parameter()
        if abs(target.basket_y - 120) > 3:
            if target.basket_y == 0:
                self.move_head(1, 1850, 880, 880, 50)
            elif target.basket_y - 120 > 0:
                self.move_head(2, self.head_vertical - 3, 880, 880, 50)
            elif target.basket_y - 120 < 0:
                self.move_head(2, self.head_vertical + 3, 880, 880, 50)
            target.basket_parameter()
            print("Basket Y = ", target.basket_y)
            print("Basket Size = ", target.basket_size)

        else:
            self.sixty_distance = sqrt(abs((3600 * six) / target.basket_size))
            self.ninty_distance = sqrt(abs((8100 * nine) / target.basket_size))
            if (six + nine) / 2 > target.basket_size:
                self.distance_new = abs(self.sixty_distance)
            else:
                self.distance_new = abs(self.ninty_distance)
            motor.throw_ball_strength()
            print("Basket vertical = ", motor.head_vertical)
            print("Basket size = ", target.basket_size)
            print("Distance_60 = ", self.sixty_distance)
            print("Distance_90 = ", self.ninty_distance)
            print("Distance_fin = ", self.distance_new)

    def imu_yaw_reset(self, fix, error_imu):
        self.yaw_fix = send.imu_value_Yaw - fix  # imu_value_yaw 當前yaw值
        if abs(self.yaw_fix) > error_imu:  # fix要修去的值
            if self.yaw_fix > 0:  # yaw_fix yaw要修的值
                print("turn right")  # error_imu 容忍範圍的誤差
                motor.MoveContinuous(right_correct[0], right_correct[1], right_correct[2], 500, 500, 1)
            elif self.yaw_fix < 0:
                print("turn left")
                motor.MoveContinuous(left_correct[0], left_correct[1], left_correct[2], 500, 500, 1)

        elif abs(self.yaw_fix) < error_imu:
            self.step_jump = True

    def bodyauto_close(self, next_state):
        # if send.is_start == True :   # 0 stop  1 go
        if self.now_state == next_state:

            pass
        elif self.now_state != next_state:
            send.sendBodyAuto(0,0,0,0,1,0)
            time.sleep(1)
            self.now_state = next_state


if __name__ == '__main__':

    target = target_location()
    motor = motor_move()
    step = 'begin'

    cnt = 3
    i, x = 0, 0
    
    correct = [-1300, -400, 0]    
    level_left_correct = [-1300, 900, 0] 
    level_right_correct = [-1300, -3000, 0] 
    left_correct = [-1500, -400, 3] 
    right_correct = [-1500, -400, -3] 
    
    #                  x , y , theta
    rotate_mistake = 50
    kick_degree , kick_error = 2825, 30

    # 第一次小踢
    kick_degree2 , kick_error2 = 2740, 30
    #直接射門
    kick_degree_mistake = 50

    over = 0
    fucking_obs = 3

    
    
    try:
        while not rospy.is_shutdown():
            if send.is_start== True:
                time.sleep(0.05)
                send.drawImageFunction(1, 0, 160, 160, 110, 130, 255, 255, 255)
                send.drawImageFunction(2, 0, 150, 170, 120, 120, 255, 255, 255)  # 對球中心線
                
                target.ball_parameter()
                target.line_parameter()

#=============================================================================================================  開始
#---------------open_ball----------------------open_ball-------------------open_ball---------------open_ball--------------open_ball
                if step == "begin":
                    motor.move_head(2,2048,880,880,50)
                    
                    send.sendSensorReset()
                    time.sleep(0.5)
                    print(send.imu_value_Yaw,send.imu_value_Pitch,send.imu_value_Roll)
                    
                    3
                    
                    step = "test"
                    
                    
                elif step == "test":
                    # send.sendBodySector(1218)
                    # send.sendBodySector(1820)  #縮右腳
                    # motor.trace_revise(target.ball_x, target.ball_y, 25)
                    # print('ver', motor.head_vertical, "hor", motor.head_horizon)
                    time.sleep(0.2)
                    
                    motor.move_head(2,2770,880,880,50)
                    time.sleep(1)
                    motor.bodyauto_close(1)
                    

                    time.sleep(0.1)
                    # print(target.obs_size_left,target.obs_size_right)
                    step = "open_ball.search_to_ball"  
                    # step = "obs.avoid_obs"
                    
                    # time.sleep(0.05)
                    # step = "open_ball.search_to_belif step =="obs.obs_before_start":
                #     target.obs_parameter() 
                #     if target.obs_y_max_left < 120 or target.obs_y_max_right < 120 :
                #         print("go ahead")
                #         motor.MoveContinuous(500 + correct[0], 0 + correct[1], 0 + correct[2], 100, 100,1)
                #     else :
                #         motor.move_head(2, 2600, 880, 880, 30)#頭往下
                #         print("obs.obs_start")
                #         time.sleep(2)
                #  all"
                    # step = "obs.search_to_ball"
                    # step = "open_ball.walk_to_ball"
                # if (cnt > 0 ):
                #     motor.view_move
                #     if (target.obs_size > 4000):
                #         motor.trace_revise(target.obs_x,target.obs_y_max,30)

                elif step == "open_ball.search_to_ball":  #開球是往左平移
                    send.drawImageFunction(8, 0,175, 175, 0, 320, 255, 255, 255)  # 對球中心線
                    print(target.ball_size)
                    target.ball_parameter()
                    
    
                    if cnt > 0:
                    
                        if target.ball_x_min < 175:
                            print("go left")
                            motor.MoveContinuous(level_left_correct[0], level_left_correct[1], level_left_correct[2], 500, 500, 1)
                            cnt = 3  #要讓球最小值離開過三次界線才跳出 預防步態不穩
                        else:
                            cnt -= 1
                    else:
                        cnt = 2
                        time.sleep(0.2)
                        step = "open_ball.trace_ball"

                
                        

                elif step == 'open_ball.trace_ball':
                    target.ball_parameter()
                    # print("open rotating")  # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # motor.bodyauto_close(1)
                    print('ver', motor.head_vertical, "hor", motor.head_horizon)

                    motor.trace_revise(target.ball_x, target.ball_y, 50)     #追蹤球

                    if abs(motor.head_vertical - kick_degree) <= kick_error:     #跟球距離洽當就踢   20 要再側
                        print("small kick")  #小踢
                        cnt = cnt -1

                    elif abs(send.imu_value_Yaw) > 5 :
                            motor.imu_yaw_reset(0,5)
                            print("reset ")
                            cnt = 2
                
                    
                    elif abs(motor.head_vertical - kick_degree) > kick_error:     #如果太遠
                        if motor.head_vertical - kick_degree < kick_error :
                            motor.MoveContinuous(correct[0]+2200,correct[1], correct[2], 500, 500, 1)
                            print("前進") 
                            cnt = 2
                        elif motor.head_vertical - kick_degree > kick_error :
                            motor.MoveContinuous(correct[0]-300,correct[1], correct[2], 500, 500, 1)
                            print("後退")  
                            cnt = 2

                    if cnt ==  0: 
                        motor.bodyauto_close(0)
                        time.sleep(4)
                        send.sendBodySector(7770)
                        
                        time.sleep(2)
                        motor.move_head(2,2600,880,880,50)
                        motor.move_head(1,2048,880,880,50)
                        time.sleep(6)
                        motor.bodyauto_close(1)
                        step ="obs.imu_reset_obs"
                        cnt = 3
                elif step == "obs.imu_reset_obs":
                    motor.imu_yaw_reset(0,5) 
                    if motor.step_jump == True:
                        step = "obs.avoid_obs"
                        time.sleep(2)
                        motor.step_jump = False

                elif step == "obs.avoid_obs":
                    # motor.bodyauto_close(1)
                    send.drawImageFunction(10, 0, 80, 80, 0, 320, 255, 0, 255)  # 對球中心線
                    target.obs2_parameter()
                    print(target.obs2_x_max,"dfdfdfd")
                    if cnt > 0:
                        if target.obs2_x_max > 65:
                            print("go righthshfjgjgjg") #右平移
                            motor.MoveContinuous(level_right_correct[0], level_right_correct[1], level_right_correct[2], 500, 500, 1)
                            cnt = 3  #要讓球最小值離開過三次界線才跳出 預防步態不穩
                        else:
                            cnt -= 1
                    else:
                        cnt = 2
                        time.sleep(0.2)
                        motor.move_head(1,1900,880,880,50)
                        motor.move_head(2,2400,880,880,50)
                        time.sleep(0.2)
                        print("------")
                        step = "obs.search_to_ball"

#======================================================================================================================
#--------------------------obs---------------------obs-----------------------obs--------------------obs------------------

                elif step == "obs.search_to_ball":#小踢完後收尋球確認球的位置
                    motor.MoveContinuous(correct[0],correct[1], correct[2], 500,500,1) #原地踏步

                    target.ball_parameter()
                    if target.ball_size < 800:  # 500要測
                        # motor.view_move(2448, 1648, 2550,2250, 70, 0.05)
                        # bbbbbbbbbbb
                        motor.view_move(2448, 1648, 2550,2200, 70, 0.05)
                        # sssssssssss
                        time.sleep(0.05)
                        target.ball_parameter()
                    elif target.ball_size > 800:
                        time.sleep(0.5)
                        step = 'obs.trace_ball'   #小踢完確認球在哪
                        

                elif step == 'obs.trace_ball':  # 持續盯著球
                    
                    target.ball_parameter()
                    if abs(target.ball_x - 160) > 8 or abs(target.ball_y - 120) > 6:
                        target.ball_parameter()
                        motor.trace_revise(target.ball_x, target.ball_y, 50)
                        print("obs_trace")  # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        time.sleep(0.05)
                    else:
                        time.sleep(0.2)
                        
                        # motor.MoveContinuous(correct[0],correct[1], correct[2], 100, 100, 1) #原地踏步
                        time.sleep(0.2)
                        step = 'obs.walk_to_ball'
#=====================================================================================================================

                elif step == 'obs.walk_to_ball':
                    time.sleep(0.05)
                    target.ball_parameter()
                    target.obs_parameter()
                    motor.trace_revise(target.ball_x, target.ball_y, 50)
                    send.drawImageFunction(3, 1, target.ball_x_min, target.ball_x_max, target.ball_y_min,target.ball_y_max, 255, 0, 255) #對球畫框定位球位置
                    send.drawImageFunction(5, 0, 100, 100, 0, 240, 100, 255, 0)  # 左(green)
                    send.drawImageFunction(6, 0, 220, 220, 0, 240, 100, 255, 0)  # 右(green)
                    print("left_obs",target.ball_x_min - target.obs_x_max_left)
                    print("right_obs",target.obs_x_min_right - target.ball_x_max)
                    # 畫不出線不知道就重開！！
                    if motor.head_horizon > 2650:
                        rotate_mistake = 110
                
                    if motor.step_jump == True :
                        motor.step_jump = False
                        motor.move_head(2, 2500, 880, 880, 30)#頭往下
                        print("obs.obs_before_start")
                        time.sleep(0.5)
                        motor.bodyauto_close(0)
                        step = "obs."

                    elif target.obs_size_left > 0 or target.obs_size_right > 0:
                        if target.obs_x_max_left > 100 :
                            print("turn right obsobsobs")
                            motor.MoveContinuous(right_correct[0]+2000, right_correct[1], right_correct[2], 500, 500, 1)
                            
                        elif target.obs_x_min_right < 220 and target.obs_x_max_right != 0:
                            print('turn left obsobsobs')
                            motor.MoveContinuous(left_correct[0]+2000, left_correct[1], left_correct[2], 500, 500, 1)
                        else:
                            if abs(motor.head_horizon - 2048) > rotate_mistake:#純轉向球
                                motor.body_trace_rotate(2048,rotate_mistake)
                                print("motor.head_vertical=========", motor.head_vertical)

                            elif abs(motor.head_vertical - kick_degree2) > kick_error2: 
                                target.ball_parameter()
                                motor.body_trace_straight(kick_degree2 ,kick_error2)

                            elif abs(motor.head_vertical - kick_degree2) < kick_error2:
                                motor.step_jump = True
                                print("hi")

                    elif(target.obs_size_left <= 0 and target.obs_size_right <= 0):
                        if abs(motor.head_horizon - 2048) > rotate_mistake:#純轉向球
                            motor.body_trace_rotate(2048,rotate_mistake)
                            print("motor.head_vertical=========", motor.head_vertical)

                        elif abs(motor.head_vertical - kick_degree2) > kick_error2: 
                            target.ball_parameter()
                            motor.body_trace_straight(kick_degree2 ,kick_error2)

                        elif abs(motor.head_vertical - kick_degree2) < kick_error2:
                            motor.step_jump = True
                            print("hi")
                    
                    

                        

            
                # elif step =="obs.obs_before_start":
                #     target.obs_parameter() 
                #     if target.obs_y_max_left < 120 or target.obs_y_max_right < 120 :
                #         print("go ahead")
                #         motor.MoveContinuous(500 + correct[0], 0 + correct[1], 0 + correct[2], 100, 100,1)
                #     else :
                #         motor.move_head(2, 2600, 880, 880, 30)#頭往下
                #         print("obs.obs_start")
                #         time.sleep(2)
                #         step = "obs.obs_start"


                
                        

                elif step == 'obs.obs_check':
                    target.obs_parameter()
                    motor.move_head(2, 2800, 880, 880, 30) 
                    step = "ball.search_to_ball"

#===========================================================================================================================
#------------------ball-----------------ball-------------------ball---------------------ball------------------

                elif step == "ball.search_to_ball":
                    if target.ball_size < 1000:
                        motor.view_move(2648, 1498, 2750, 2550, 70, 0.05) #避障後搜球確認球位置
                        time.sleep(0.05)
                        target.ball_parameter()
                        print("  ball => x:", target.ball_x, " y:", target.ball_y, " size:", target.ball_size)
                    elif target.ball_size > 1000:
                        time.sleep(1)
                        print("hi")
                        step = 'ball.watch_ball'

                elif step == 'ball.watch_ball':
                    if abs(target.ball_x - 160) > 8 or abs(target.ball_y - 120) > 6:
                        target.ball_parameter()
                        motor.trace_revise(target.ball_x, target.ball_y, 25)
                        time.sleep(0.05)
                    else:
                        print("find center")
                        motor.bodyauto_close(1)
                        

                        step = 'ball.walk_to_ball'

                elif step == 'ball.walk_to_ball':
                    target.ball_parameter()

                    print('ver', motor.head_vertical, "hor", motor.head_horizon)

                    motor.trace_revise(target.ball_x, target.ball_y, 25)
                    print("open walking")
                    
                    if abs(motor.head_horizon - 2048) > rotate_mistake:
                        target.ball_parameter()
                        motor.body_trace_rotate(2048,rotate_mistake)
                        print("motor.head_vertical=========", motor.head_vertical)

                    else:  # 1320是調球的距離150是誤差
                        target.ball_parameter()
                        motor.body_trace_straight(kick_degree2 ,kick_error2)
#===================================3/6=====================3/6============================3/6==============================
                        if motor.step_jump == True : # step_jump 連續之跳出
                                step = "kick.imu_reset"
                                print("imu_reset")
                                motor.step_jump = False
#===========================================================================================================================
                elif step == "kick.imu_reset":  # IMU reset 平行球框
                    print(send.imu_value_Yaw,send.imu_value_Pitch,send.imu_value_Roll)
                    motor.step_jump == True
                    # motor.imu_yaw_reset(0,5)
                    if motor.step_jump == True:
                       
                        motor.step_jump = False
                        print("close walking")  
                        motor.bodyauto_close(0)
                        send.sendBodySector(8787)
                        time.sleep(1)
                        send.sendBodySector(7878)
                        time.sleep(1)
                        step = "xx"
                elif step == 'xx':
                    target.ball_parameter()
                    # print("open rotating")  # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    # motor.bodyauto_close(1)
                    print('ver', motor.head_vertical, "hor", motor.head_horizon)

                    motor.trace_revise(target.ball_x, target.ball_y, 50)     #追蹤球

                    if abs(motor.head_vertical - kick_degree) <= kick_error:     #跟球距離洽當就踢   20 要再側
                        print("small kick")  #小踢
                        cnt = cnt -1

                    elif abs(send.imu_value_Yaw) > 5 :
                            motor.imu_yaw_reset(0,5)
                            print("reset ")
                            cnt = 2

                    
                    elif abs(motor.head_vertical - kick_degree) > kick_error:     #如果太遠
                        if motor.head_vertical - kick_degree < kick_error :
                            motor.MoveContinuous(correct[0]+2200,correct[1], correct[2], 500, 500, 1)
                            print("前進") 
                            cnt = 2
                        elif motor.head_vertical - kick_degree > kick_error :
                            motor.MoveContinuous(correct[0]-300,correct[1], correct[2], 500, 500, 1)
                            print("後退")  
                            cnt = 2

                    if cnt ==  0: 
                        motor.bodyauto_close(0)
                        send.sendBodySector(7771)
                        time.sleep(4)
 
#=============================================================================================================================================

                elif step == "kick.search_to_ball":  # 踢球前確認球在畫面內
                    if target.ball_size < 1000:
                        motor.view_move(2548, 1548, 2750,2550, 70, 0.05)
                        time.sleep(0.05)
                        target.ball_parameter()
                        print("  ball => x:", target.ball_x, " y:", target.ball_y, " size:", target.ball_size)
                    elif target.ball_size > 1000:
                        step = 'kick.trace_ball'

                elif step == 'kick.trace_ball':
                    if abs(target.ball_x - 160) > 8 or abs(target.ball_y - 120) > 6:
                        target.ball_parameter()
                        motor.trace_revise(target.ball_x, target.ball_y, 25)
                        print("open walking")  # !!!!!!!!!!!!!!!!!!!!!!!!!!!
                        # motor.bodyauto_close(1)

                    else:
                        print("fini")
                        step = 'kick.walk_to_ball'

                elif step == 'kick.walk_to_ball':
                    target.ball_parameter()
                    print('ver', motor.head_vertical, "hor", motor.head_horizon)
                    motor.trace_revise(target.ball_x, target.ball_y, 25)
                    motor.body_trace_level(2048,2850,50,70)

                    if motor.step_jump == True:
                        step = 'kick.ready_kick_ball'
                        print("ready to kick.ball")

                elif step == 'kick.ready_kick_ball':
                    print("------ score ------")
                    step = "finish"
#=================================================================================================================================
            else:
                # if step != 'begin' and (send.DIOValue == 8 or send.DIOValue == 9 or send.DIOValue == 10 or send.DIOValue == 12 or send.DIOValue == 13
                #                      or send.DIOValue == 11 or send.DIOValue == 15 or send.DIOValue == 24 or send.DIOValue == 27 or send.DIOValue == 31):
                if step != 'begin' and send.is_start == False:  # gazebo test
                    motor.bodyauto_close(0)
                    print("bodyauto_close", motor.bodyauto_close)  # !!!!!!!!!!
                    target = target_location()
                    motor = motor_move()
                    step = 'begin'
                    send.sendHeadMotor(1, 2048, 30)  # reset head
                    send.sendHeadMotor(2, 2048, 30)
                    time.sleep(1)
                    send.sendBodySector(29)  # reset body
                    print("-------------------reset and stoping-------------------------")
                    print("-------------------reset and stoping-------------------------")
                    print("好棒棒")
                    print("")
                    print("◢███◣。。。。。。◢███◣")
                    print("▇▇□██。。。。。。██□██")
                    print("  ◥███◤◢████◣◥███◤")
                    print("◢█。。。。。。。。。。█◣")
                    print("█。╔╗。。。。。。。╔╗。█")
                    print("█。∥●。。。╭╮。。。∥●。█")
                    print("█。╚╝。。。╰╯。。。╚╝。█")
                    print("   ◥▆▆▆▆▆▆▆▆▆▆▆▆▆▆")
                    time.sleep(0.05)
                    print("\n")
                    print(" ┌─╮◆╭═┐╭═┐╭═┐◆╭─┐")

                    print("│┌╯◆║加║║油║║囉║◆╰┐│")

                    print(" ╰╯↘◆└═╯└═╯└═╯◆↙╰╯")

                    print("\n")
                    print("..../\„./\...../\„./\ ")
                    print("... (=';'=)....(=';'=) ♥♥")
                    print("..../*♥♥**\ ♥  /*♥♥**\ ")
                    print(".(.| |..| |.)(.| |..| |.)♥")



















    except rospy.ROSInterruptException:
        pass

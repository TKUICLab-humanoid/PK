#!/usr/bin/env python
#coding=utf-8
from cmath import sqrt
from re import T
import time
from traceback import print_tb


import numpy as np
import rospy
from Python_API import Sendmessage
send=Sendmessage()

class target_location():
    def __init__(self):
        self.color_mask_subject_blue = 0
        self.color_mask_subject_orange = 0
        self.color_mask_subject_white = 0
        #======================================
        self.obs_x_left = 0
        self.obs_y_left = 0
        self.obs_size_left = 0
        self.obs_x_min_left = 0
        self.obs_y_min_left = 0
        self.obs_x_max_left = 0
        self.obs_y_max_left = 0
         
        self.obs_x_right = 0
        self.obs_y_right = 0
        self.obs_width_right = 0
        self.obs_height_right = 0
        self.obs_x_min_right = 0
        self.obs_y_min_right = 0
        self.obs_x_max_right = 0
        self.obs_y_max_right = 0
        self.obs_size_right = 0
        #==============================================
        self.line_x = [0,0,0,0,0,0,0,0,0,0]
        self.line_y = [0,0,0,0,0,0,0,0,0,0]
        self.line_x_max = [0,0,0,0,0,0,0,0,0,0]
        self.line_y_max = [0,0,0,0,0,0,0,0,0,0]
        self.line_x_min = [0,0,0,0,0,0,0,0,0,0]
        self.line_y_min = [0,0,0,0,0,0,0,0,0,0]
        self.line_size = [0,0,0,0,0,0,0,0,0,0]
        #===============================================
        self.ball_x = 0
        self.ball_y = 0
        self.ball_size = 0
        self.ball_x_min = 0
        self.ball_y_min = 0
        self.ball_x_max = 0
        self.ball_y_max = 0
        #==============================================
        
        
    def obs_parameter(self):
        self.color_mask_subject_blue = send.color_mask_subject_cnts[2]
        for j in range (self.color_mask_subject_blue):
            if send.color_mask_subject_XMin[2][j] > 160:
                if send.color_mask_subject_size [2][j] > 1000:
                    self.obs_x_right =  send.color_mask_subject_X [2][j]
                    self.obs_y_right = send.color_mask_subject_Y [2][j]
                    self.obs_size_right = send.color_mask_subject_size [2][j]
                    self.obs_x_min_right = send.color_mask_subject_XMin[2][j] 
                    self.obs_y_min_right = send.color_mask_subject_YMin[2][j] 
                    self.obs_x_max_right = send.color_mask_subject_XMax[2][j] 
                    self.obs_y_max_right =send.color_mask_subject_YMax[2][j]
                    self.obs_width_right = send.color_mask_subject_Width[2][j]
                    self.obs_height_right = send.color_mask_subject_Height[2][j]
                else :
                    self.obs_x_right = 0
                    self.obs_y_right = 0
                    self.obs_width_right = 0
                    self.obs_height_right = 0
                    self.obs_x_min_right = 0
                    self.obs_y_min_right = 0
                    self.obs_x_max_right = 0
                    self.obs_y_max_right = 0
                    self.obs_size_right = 0
            
            elif  send.color_mask_subject_XMin[2][j] < 160:
                if send.color_mask_subject_size [2][j] > 1000 :
                    self.obs_x_left =  send.color_mask_subject_X [2][j]
                    self.obs_y_left = send.color_mask_subject_Y [2][j]
                    self.obs_size_left = send.color_mask_subject_size [2][j]
                    self.obs_x_min_left = send.color_mask_subject_XMin[2][j] 
                    self.obs_y_min_left = send.color_mask_subject_YMin[2][j] 
                    self.obs_x_max_left = send.color_mask_subject_XMax[2][j] 
                    self.obs_y_max_left =send.color_mask_subject_YMax[2][j]
                    self.obs_widtd_left = send.color_mask_subject_Width[2][j]
                    self.obs_height_left = send.color_mask_subject_Height[2][j]
                else :
                    self.obs_x_left = 0
                    self.obs_y_left = 0
                    self.obs_size_left = 0
                    self.obs_x_min_left = 0
                    self.obs_y_min_left = 0
                    self.obs_x_max_left = 0
                    self.obs_y_max_left = 0

            

    
        

    def ball_parameter(self):
        self.color_mask_subject_orange = send.color_mask_subject_cnts[0]
        for j in range (self.color_mask_subject_orange):
            if send.color_mask_subject_size [0][j] > 100 :
                self.ball_x =  send.color_mask_subject_X [0][j]
                self.ball_y = send.color_mask_subject_Y [0][j]
                self.ball_size = send.color_mask_subject_size [0][j]
                self.ball_x_min = send.color_mask_subject_XMin[0][j] 
                self.ball_y_min = send.color_mask_subject_YMin[0][j] 
                self.ball_x_max = send.color_mask_subject_XMax[0][j] 
                self.ball_y_max =send.color_mask_subject_YMax[0][j]
            else :
                self.ball_x = 0
                self.ball_y = 0
                self.ball_size = 0            
                self.ball_x_min = 0
                self.ball_y_min = 0
                self.ball_x_max = 0
                self.ball_y_max = 0
   
    def line_parameter(self):
        self.color_mask_subject_white = send.color_mask_subject_cnts[6]
        for j in range (self.color_mask_subject_white):
            if send.color_mask_subject_size [6][j] > 1000 :
                self.line_x[j] =  send.color_mask_subject_X [6][j]
                self.line_y[j] = send.color_mask_subject_Y [6][j]
                self.line_size[j] = send.color_mask_subject_size [6][j]
                self.line_x_min[j] = send.color_mask_subject_XMin[6][j] 
                self.line_y_min[j] = send.color_mask_subject_YMin[6][j] 
                self.line_x_max[j] = send.color_mask_subject_XMax[6][j] 
                self.line_y_max[j] =send.color_mask_subject_YMax[6][j]
            else :
                self.line_x[j] = 0
                self.line_y[j] = 0
                self.line_size[j] = 0            
                self.line_x_min[j] = 0
                self.line_y_min[j] = 0
                self.line_x_max[j] = 0
                self.line_y_max[j] = 0
        
class motor_move():
    #motor = motor_move()
    target = target_location()
    motor = target_location()
    def __init__(self):
        self.head_horizon = 2048                #頭部水平刻度
        self.head_vertical = 2048               #頭部垂直刻度
        self.waist_position = 2048              #腰當下的刻度
        self.head_horizon_flag = 0              #找目標物的旗標
        self.x_differ = 0                       #目標與中心x差距
        self.y_differ = 0                       #目標與中心y差距
        self.x_degree = 0                       #目標與中心x角度
        self.y_degree = 0 
        self.body_level = 0                       #目標與中心y角度
        self.body_rotate = 0                  #身體要旋轉多少？？？
        self.body_straight = 0
        self.found = False
        self.catch = False
        self.now_state = 0
        self.Move_waist = 0                     #修腰
        self.MoveY = 0                          #轉腰修正頭的高度
        self.NowX = 0                           #現在要移動的x量
        self.NowY = 0                           #現在要移動的y量
        self.NowTheta = 0                       #現在要旋轉的theta量
        self.throw_strength = 0                 #不知道
        self.sixty_distance = 0
        self.ninty_distance = 0
        self.detect = False
        self.distance_new = 0
        self.now_switch_state = "begin"
        self.yaw_fix = 0
        send.imu_value_Yaw = 0
        self.step_jump = False
       
    #開球： 平移兩部後朝向球 小踢

    def move_head(self,ID,Position,max_head_horizon_size,max_head_vertical_size,Speed):
        send.sendHeadMotor(ID,Position,Speed)
        if ID == 1 :
            self.head_horizon =  Position
            if abs(self.head_horizon - 2048) > max_head_horizon_size :
                if (self.head_horizon - 2048 ) > 0 :
                    self.head_horizon = 2048 + max_head_horizon_size
                elif (self.head_horizon - 2048 ) < 0 :
                    self.head_horizon = 2048 - max_head_horizon_size

        else :
            self.head_vertical = Position
            if abs(self.head_vertical - 2048) > max_head_vertical_size :
                    if (self.head_vertical - 2048 ) > 0 :
                        self.head_vertical = 2048 + max_head_vertical_size
                    elif (self.head_vertical - 2048) < 0 :
                        self.head_vertical = 2048 - max_head_vertical_size

    
    

    def view_move(self,right_place,left_place,up_place,down_place,speed,delay):

        if self.head_horizon_flag == 1 :
            if self.head_horizon > left_place:
                self.move_head(1,self.head_horizon,880,880,speed)
                self.head_horizon = self.head_horizon - speed
                time.sleep(delay) 
            else :
                self.move_head(2,up_place,880,880,100)
                self.head_horizon_flag = 0                   
        else :                
            if  self.head_horizon < right_place:
                self.move_head(1,self.head_horizon,880,880,speed)
                self.head_horizon = self.head_horizon + speed
                time.sleep(delay) 
            else :
                self.move_head(2,down_place,880,880,100)
                self.head_horizon_flag = 1
    
    def trace_revise(self,x_target,y_target,speed) :
        #if abs(x_target) > 0 and abs(y_target) > 0:
        self.x_differ =  x_target - 160 
        self.y_differ =  y_target - 120 
        self.x_degree = self.x_differ * (70.42 / 320)
        self.y_degree = self.y_differ * (43.3 / 240)
        self.move_head(1, self.head_horizon - round(self.x_degree * 4096 / 360 *0.15),880,880,speed)
        self.move_head(2, self.head_vertical - round(self.y_degree * 4096 / 360 *0.15),880,880,speed)
        time.sleep(0.05)

    def body_trace_rotate(self,degree) :
        self.body_rotate = self.head_horizon - 2048
        if self.body_rotate > degree :
            motor.MoveContinuous(left_correct[0],left_correct[1],left_correct[2],100,100,2)
            print( "go left = ",self.body_rotate)
            time.sleep(0.05)
        elif self.body_rotate < -degree :
            motor.MoveContinuous(right_correct[0],right_correct[1],right_correct[2],100,100,2)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            print( "go right = ",self.body_rotate)
            time.sleep(0.05)

    def body_trace_level(self,degree_level,degree_straight,error_level,error_straight) : #要的水平角度  要的直走角度 容忍的水平角度誤差 容忍的直走角度
        self.body_level = self.head_horizon - degree_level
        self.body_straight = self.head_vertical - degree_straight
        if abs(self.body_rotate) > error_level :
            if self.body_rotate > 0 : #左正右負
                print( "go left level ")
                time.sleep(0.05)
            elif self.body_rotate < 0 :
                print( "go right level ")
                time.sleep(0.05)   
        elif abs(self.body_straight) > error_straight :
            if self.body_straight > 0 :
                print( "go ahead ")
                time.sleep(0.05)
            elif self.body_straight < 0 :
                print( "go back ")
                time.sleep(0.05)   
        else: 
            motor.step_jump  = True 

    def body_obs_rotate(self,degree) :
        self.body_rotate = self.head_horizon - 2048
        if self.body_rotate > degree :
            motor.MoveContinuous(left_correct[0],left_correct[1],left_correct[2],100,100,2)
            print( "go left = ",self.body_rotate)
            time.sleep(0.05)
        elif self.body_rotate < -degree :
            motor.MoveContinuous(right_correct[0],right_correct[1],right_correct[2],100,100,2)#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            print( "go right = ",self.body_rotate)
            time.sleep(0.05)

    def body_trace_straight(self,degree,ball_degree) :
        
        if (self.head_vertical - degree)  > ball_degree :
            motor.MoveContinuous(1900+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
            print( "go ahead  ",self.head_vertical)
            time.sleep(0.05)
        
        elif (self.head_vertical - degree) < -ball_degree :
            motor.MoveContinuous(-1500+correct[0],0+correct[1],0+correct[2],100,100,2)
            print( "go back = ",self.head_vertical)
            time.sleep(0.05)
        elif  abs(self.head_vertical - degree) <= ball_degree :
            #send.sendBodyAuto(0,0,0,0,1,0)
            self.step_jump = True
            time.sleep(3)
            
    # def waist_revise(self):
            
    #             time.sleep(1)
    #             send.sendSingleMotor(9,round(self.body_rotate ),30)
    #             print("meowmeowmeoemeowmeow",self.body_rotate)
    #             time.sleep(0.05)

    def WaistFix(self, Target_X, Target_Y, TargetXCenter, TargeYCenter):#轉腰調整Basket.X與Baskethead_verticalBaseLine的誤差
        # self.MoveW = round((TargetXCenter - Target_X)*0.5)
        # self.MoveY = TargeYCenter - Target_Y

        self.MoveW = TargetXCenter - Target_X
        if self.MoveW > 15:
            self.MoveW = 15
        
        self.waist_reset((self.waist_position + self.MoveW), 30)
        self.move_head(2, self.head_vertical - round(self.y_degree * 4096 / 360 *0.15),880,880,20)
        time.sleep(0.15)
        
        # time.sleep(0.2)


     ####################################### basket degree version #######################################

    def body_trace_basket_straight_2(self,degree,basket_error) :
        
        if self.head_vertical - degree > basket_error  and self.head_vertical > 1980 :
            motor.MoveContinuous(2600+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead bbbb to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif self.head_vertical - degree > basket_error and  self.head_vertical < 1980:
            motor.MoveContinuous(1000+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead sss to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)    
        elif self.head_vertical - degree < basket_error and abs(self.head_vertical - degree) > basket_error:
            motor.MoveContinuous(-800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!
    
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)
        elif abs(self.head_vertical - degree) < basket_error : 
            print( "--------------------stop at the basket----------------------",self.head_vertical - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(1)
            motor.move_head(1,1798,880,880,30)
            time.sleep(0.8)
            send.sendBodySector(3)   #準備上籃之動作一（舉手...）111111111111111111111111111111111111111111111111111111111`
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(2.5)
            self.found = True
            self.catch = True
    
     #####################################################################################################

    def body_trace_basket_straight_3(self,basket_size,basket_error) :
        
        if target.basket_size - basket_size < -basket_error and target.basket_size > 2100:
            send.sendContinuousValue(300+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)
        elif target.basket_size - basket_size < -basket_error and target.basket_size < 2100:
            send.sendContinuousValue(1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!
            print( "--------------------go ahead to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif target.basket_size - basket_size > basket_error and target.basket_size > 2400:
            send.sendContinuousValue(-1200+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)   
        elif target.basket_size - basket_size > basket_error and target.basket_size < 2400:
            send.sendContinuousValue(-800+correct[0],0+correct[1],0,0+correct[2],0)#!!!!!!!!!!!!!!!!!
            print( "--------------------go back from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)
        elif abs(target.basket_size - basket_size) < basket_error :
            print( "--------------------stop at the basket----------------------",self.head_horizon - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(0.5)
            motor.move_head(1,2048,880,880,30)
            time.sleep(0.2)
            # send.sendBodySector(3)   #33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(2)
            self.found = True
            self.catch = True
    
    def body_trace_basket_straight_5(self,degree,basket_error) :
        
        if self.head_vertical - degree < basket_error  and abs(self.head_vertical - degree) > basket_error and self.head_vertical < 2150 :
            motor.MoveContinuous(-1500+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go back bbbb to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)

        elif self.head_vertical - degree < basket_error  and abs(self.head_vertical - degree) > basket_error and self.head_vertical > 2150:
            motor.MoveContinuous(-1000+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!!
            print( "--------------------go back sss to basket---------------------  ",self.head_vertical)
            time.sleep(0.05)    
        elif self.head_vertical - degree > basket_error:
            motor.MoveContinuous(800+correct[0],0+correct[1],0+correct[2],100,100,2)#!!!!!!!!!!!!!!
    
            print( "--------------------go ahead from basket-------------------- ",self.head_vertical)
            time.sleep(0.05)
        elif abs(self.head_vertical - degree) < basket_error : 
            print( "--------------------stop at the basket----------------------",self.head_vertical - 2048)
            #send.sendBodyAuto(0,0,0,0,1,0)
            motor.bodyauto_close(0)
            target.basket_parameter()
            
            time.sleep(1)
            send.sendBodySector(5301)
            time.sleep(0.8)
            
            print("-------------------------send.sendBodySector(3)------------------------------")
            time.sleep(2.5)
            self.found = True
            self.catch = True
    
    
    def MoveContinuous(self ,ExpX ,ExpY ,ExpTheta ,AddX ,AddY ,AddTheta) :
        if abs(self.NowX - ExpX) < AddX:
            self.NowX = ExpX
        else:
            if self.NowX < ExpX :
                self.NowX += AddX
            elif self.NowX > ExpX :
                self.NowX -= AddX
            else:
                pass

        if abs(self.NowY - ExpY) < AddY:
            self.NowY = ExpY
        else:
            if self.NowY < ExpY :
                self.NowY += AddY
            elif self.NowY > ExpY :
                self.NowY -= AddY
            else:
                pass

        if abs(self.NowTheta - ExpTheta) < AddTheta:
            self.NowTheta = ExpTheta
        else:
            if self.NowTheta < ExpTheta :
                self.NowTheta += AddTheta
            elif self.NowTheta > ExpTheta :
                self.NowTheta -= AddTheta
            else:
                pass

        # print("NowX, NowY, NowTheta = ", self.NowX, ", ", self.NowY, ", ", self.NowTheta)
        send.sendContinuousValue(self.NowX, self.NowY, 0, self.NowTheta , 0)



    def test_distance(self):
        
        target.basket_parameter()
        #if target.basket_size >= 300 and abs(motor.head_horizon-2048) < 600:

        if abs(target.basket_x - 160) > 5  or abs(target.basket_y - 120) > 4 :
            motor.trace_revise(target.basket_x,target.basket_y,25)
            target.basket_parameter() 
            
            time.sleep(0.05) 
            
        else :
            print("Basket Y = ",target.basket_size)
                


    


    def basket_distance(self,six,nine):
        print("target.basket_y - 120",target.basket_y - 120)
        self.move_head(1,2048,880,880,50)
        target.basket_parameter()
        if abs(target.basket_y - 120) > 3 :
            if target.basket_y == 0 :
                self.move_head(1,1850,880,880,50)
            elif target.basket_y - 120 > 0 :
                self.move_head(2,self.head_vertical - 3,880,880,50)
            elif target.basket_y - 120 < 0 :
                self.move_head(2,self.head_vertical + 3,880,880,50)
            target.basket_parameter()
            print("Basket Y = ",target.basket_y)
            print("Basket Size = ",target.basket_size)
        
        else  :
            self.sixty_distance = sqrt(abs((3600*six)/target.basket_size))
            self.ninty_distance = sqrt(abs((8100*nine)/target.basket_size))
            if ( six + nine ) / 2 > target.basket_size :
                self.distance_new = abs(self.sixty_distance)
            else :
                self.distance_new = abs(self.ninty_distance)
            motor.throw_ball_strength()
            print("Basket vertical = ",motor.head_vertical)
            print("Basket size = ",target.basket_size)
            print("Distance_60 = ",self.sixty_distance)
            print("Distance_90 = ",self.ninty_distance)
            print("Distance_fin = ",self.distance_new)

    def imu_yaw_reset(self,fix,error_imu):
        self.yaw_fix = send.imu_value_Yaw - fix  #imu_value_yaw 當前yaw值  
        if abs(self.yaw_fix) > error_imu :       #fix要修去的值 
            if self.yaw_fix > 0 :                #yaw_fix yaw要修的值
                print("turn right")              #error_imu 容忍範圍的誤差
            elif self.yaw_fix < 0 :
                print("turn left")
        elif abs(self.yaw_fix) < error_imu :
            self.step_jump = True



    def bodyauto_close(self,next_state):
        #if send.is_start == True :   # 0 stop  1 go
        if self.now_state == next_state :
            
            pass
        elif self.now_state != next_state :
            send.sendBodyAuto(0,0,0,0,1,0)
            self.now_state = next_state
                

        


if __name__ == '__main__' :
    
    target = target_location()
    motor = motor_move()
    step ='begin'
    
    cnt = 3
    i,x = 0,0
    
    correct       = [-100,0,-2]
    left_correct  = [-100,0,6]
    right_correct = [-100,0,-6]
    #                  x , y , theta 
    rotate_mistake = 400
    kick_degree = 1330
    kick_degree_mistake = 50
    
    #step_jump 連續之跳出
    try:    
        while not rospy.is_shutdown():
            if send.Web==True  :
                if step == "begin" :
                    #send.sendSensorReset()
                    # print(send.imu_value_Roll  ,
                    #       send.imu_value_Pitch ,
                    #       send.imu_value_Yaw)
                    step =  "open_ball.search_to_ball"
                    
                send.drawImageFunction(1,0,160,160,110,130,255,255,255) 
                send.drawImageFunction(2,0,150,170,120,120,255,255,255)#對球中心線
               
                target.obs_parameter() 
                target.ball_parameter() 
                target.line_parameter() 

                # print("obs_size:",target.obs_size)
                # print("ball_size:",target.ball_size)
                # print('ver',motor.head_vertical,"hor",motor.head_horizon)
                # print("obs_x",target.obs_x)
                # print("obs_y",target.obs_y)
                # print("obs_x_min",target.obs_x_min)
                # print("obs_x_max",target.obs_x_max)
                # print("obs_y_min",target.obs_y_min)
                # print("obs_y_max",target.obs_y_max)
                #send.drawImageFunction(3,1,target.obs_x-5,target.obs_x+5,target.obs_y_max-5,target.obs_y_max+5,255,0,0)
                
                # if (cnt > 0 ):
                #     motor.view_move
                #     if (target.obs_size > 4000):
                #         motor.trace_revise(target.obs_x,target.obs_y_max,30)
                if step == "open_ball.search_to_ball" :
                    motor.move_head(0,2048,800,800,30)
                    if cnt > 0 :
                        if target.ball_x_min < 170 :
                            print("go left")
                            cnt = 3
                        else :
                            cnt -= 1
                    else :
                        cnt = 2
                        step = "open_ball.trace_ball"

                elif step == "open_ball.trace_ball" :
                    if cnt > 0 :
                        if abs(target.ball_x - 160) > 10  or abs(target.ball_y - 120) > 10 :
                            target.ball_parameter() 
                            motor.trace_revise(target.ball_x,target.ball_y,25)
                            print("open_ball_trace")#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                            time.sleep(0.05)
                            cnt = 2
                        else :
                            cnt -= 1
                    else :
                        cnt = 2
                        step = "open_ball.walk_to_ball"
                
                elif  step == 'open_ball.walk_to_ball' :
                        time.sleep(0.05)
                        target.ball_parameter()
                        print("open walking")  # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        #motor.bodyauto_close(1)
                        print('ver',motor.head_vertical,"hor",motor.head_horizon)  
                        
                        motor.trace_revise(target.ball_x,target.ball_y,25)

                        if abs(motor.head_horizon-2048) > rotate_mistake  :  
                            motor.body_trace_rotate(rotate_mistake)    
                            print("motor.head_vertical=========",motor.head_vertical)
                        
                        elif abs(motor.head_vertical - kick_degree) > 5 :
                            motor.move_head(1, 1500,880,880,30)
                            step = "obs.obs_start"


                elif step == "obs.search_to_ball":
                    if target.ball_size < 500 : # 500要測  
                        motor.view_move(2698,1498,1800,1098,55,0.05)                 
                        time.sleep(0.05)
                        target.ball_parameter()  
                        print("  ball => x:",target.ball_x," y:",target.ball_y," size:",target.ball_size)
                    elif target.ball_size > 500 :
                        send.imu_value_Yaw = 0 
                        step = 'obs.trace_ball'
                
                elif step == 'obs.trace_ball' :#持續盯著球
                    if abs(target.ball_x - 160) > 10  or abs(target.ball_y - 120) > 10 :
                        target.ball_parameter() 
                        motor.trace_revise(target.ball_x,target.ball_y,25)
                        print("obs_trace")#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        time.sleep(0.05) 
                    else :
                        print("find center")
                        step = 'obs.walk_to_ball'

                elif  step == 'obs.walk_to_ball' :
                        time.sleep(0.5)
                        target.ball_parameter()
                        print('\t')
                        print("open walking")  # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        motor.bodyauto_close(1)
                        print('ver',motor.head_vertical,"hor",motor.head_horizon)  
                        
                        motor.trace_revise(target.ball_x,target.ball_y,25)

                        if abs(motor.head_horizon-2048) > rotate_mistake  :  
                            motor.body_trace_rotate(rotate_mistake)    
                            print("motor.head_vertical=========",motor.head_vertical)
                        
                        elif abs(motor.head_vertical - kick_degree) > 5 :
                            motor.move_head(1, 1500,880,880,30)
                            motor.body_trace_straight(kick_degree,kick_degree_mistake)
                            step = "obs.obs_start"
                
                elif  step == 'obs.obs_start' :
                    target.obs_parameter()
                    target.line_parameter()
                    send.drawImageFunction(4,0,0,320,100,100,255,0,0)
                    send.drawImageFunction(5,0,70,70,0,240,100,255,0)    #左(green)
                    send.drawImageFunction(6,0,250,250,0,240,100,255,0)  # 右(green)
                    send.drawImageFunction(7,0,0,320,180,180,0,0,0)  #白線界線(black)
                    #print("line",target.line_size)
                    #print("obs" , target.obs_size , target.obs_y_max , target.obs_y_min)

                    print(send.color_mask_subject_cnts[6]) 
                    # if (target.obs_y_max > 100  and target.obs_y_min < 220) and (target.obs_size != 0 ) :
                        
                    #     print(target.obs_x_max,target.obs_x_min)
                    #     if abs(target.obs_x_max - 70) <  abs(target.obs_x_max - 0 ) and 70 < target.obs_x_max < 160:    #物體位於左側
                    #         print('go right')
                    #         motor.MoveContinuous(right_correct[0],right_correct[1],right_correct[2],100,100,2)

                    #     elif abs(target.obs_x_min -250)  < abs(target.obs_x_min - 320 ) and 250 > target.obs_x_min > 160: #物體位於右側
                    #         print('go left')
                    #         motor.MoveContinuous(left_correct[0],left_correct[1],left_correct[2],100,100,2)
                        
                    #     elif (abs(target.obs_x_min -160) > 90 and abs(target.obs_x_max -160) > 90)  :
                    #         motor.MoveContinuous(100+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
                    #         print( "go ahead ssss")
                    
                    # elif target.obs_y_max < 100 and target.obs_size > 1500:
                        
                    #     motor.MoveContinuous(500+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
                    #     print( "go ahead bbbb")
                    if target.obs_size_left > 0 or  target.obs_size_right > 0:
                        if target.obs_x_max_left > 70 :
                            print("turn right")
                            motor.MoveContinuous(left_correct[0],left_correct[1],left_correct[2],100,100,2)
                        elif target.obs_x_min_right < 250 :
                            print('turn left')
                            motor.MoveContinuous(right_correct[0],right_correct[1],right_correct[2],100,100,2)
                        elif target.obs_x_max_left < 70  and target.obs_x_min_right > 250 :
                            print("go ahead")
                            motor.MoveContinuous(100+correct[0],0+correct[1],0+correct[2],100,100,2)

                    elif  target.line_y_max[0] < 180 :
                        # target.line_size[0] = 0
                        motor.MoveContinuous(100+correct[0],0+correct[1],0+correct[2],100,100,2) #!!!!!!!!!!!!!!!!!!!!!!
                        print( "go ahead ssss line")
                    
                    elif target.line_y_max[0] > 180:
                        print("get out")
                        step = "obs.obs_check"
                        print("stop walking")  # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        motor.bodyauto_close(0)
        
                            
                        
                elif  step == 'obs.obs_check' :        
                    target.obs_parameter()
                    
                    step = "ball.search_to_ball"#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!少一個ball
                        
#===========================================================================================================================

                elif step == "ball.search_to_ball":
                    if target.ball_size < 1000 :
                        motor.view_move(2698,1498,1800,1098,55,0.05)                 
                        time.sleep(0.05)
                        target.ball_parameter()  
                        print("  ball => x:",target.ball_x," y:",target.ball_y," size:",target.ball_size)
                    elif target.ball_size > 1000 :
                        step = 'ball.watch_ball'
                            
                elif step == 'ball.watch_ball' :
                    if abs(target.ball_x - 160) > 10  or abs(target.ball_y - 120) > 10 :
                        target.ball_parameter() 
                        motor.trace_revise(target.ball_x,target.ball_y,25)
                        time.sleep(0.05) 
                    else :
                        print("find center")
                        step = 'ball.walk_to_ball'

                
                elif  step == 'ball.walk_to_ball' :
                        time.sleep(0.5)
                        target.ball_parameter()            
                        print("\t")
                        print('ver',motor.head_vertical,"hor",motor.head_horizon)  
                        
                        motor.trace_revise(target.ball_x,target.ball_y,25)
                        print("open walking")
                        motor.bodyauto_close(1)
                        if abs(motor.head_horizon-2048) > rotate_mistake  :  
                            motor.body_trace_rotate(rotate_mistake)    
                            print("motor.head_vertical=========",motor.head_vertical)
                        
                        elif abs(motor.head_vertical - kick_degree) > 5 : #1320是調球的距離150是誤差
                            print(motor.head_horizon - 2048)
                            motor.body_trace_straight(kick_degree,kick_degree_mistake)#!!!!!!!!!!!!!!!!!!!!!!!!!!!球的距離夠motor.found = True
                            if motor.step_jump == True : #body_trace_level
                                step = "kick.imu_reset"
                                motor.step_jump = False
#===========================================================================================================================

                elif  step == "kick.imu_reset" :  # IMU reset 平行球框
                    motor.imu_yaw_reset(0,5)      
                    if motor.step_jump == True :
                        step = "kick.search_to_ball"
                        motor.step_jump = False
                        print("close walking")#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        motor.bodyauto_close(0)
                        print("bodyauto_close", motor.bodyauto_close)
                elif  step == "kick.search_to_ball" : #踢球前確認球位置
                    if target.ball_size < 1000 :
                        motor.view_move(2698,1498,1800,1098,55,0.05)                 
                        time.sleep(0.05)
                        target.ball_parameter()  
                        print("  ball => x:",target.ball_x," y:",target.ball_y," size:",target.ball_size)
                    elif target.ball_size > 1000 :
                        step = 'kick.watch_ball'
                
                elif step == 'kick.trace_ball' :
                    if abs(target.ball_x - 160) > 10  or abs(target.ball_y - 120) > 10 :
                        target.ball_parameter() 
                        motor.trace_revise(target.ball_x,target.ball_y,25)
                        print("open walking")#!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        motor.bodyauto_close(1)
                        print("bodyauto_close", motor.bodyauto_close)
                        time.sleep(0.05) 
                    else :
                        step = 'kick.walk_to_ball'

                elif  step == 'kick.walk_to_ball' :
                        time.sleep(0.5)
                        target.ball_parameter()            
                        print('ver',motor.head_vertical,"hor",motor.head_horizon)
                        motor.trace_revise(target.ball_x,target.ball_y,25)
                        motor.body_trace_level()
                        
                        if motor.step_jump == True :
                             step = 'kick.ready_kick_deardeer'
                             print("ready to kick.deardeer")

                        # =================================================================================================================================
            else:
                # if step != 'begin' and (send.DIOValue == 8 or send.DIOValue == 9 or send.DIOValue == 10 or send.DIOValue == 12 or send.DIOValue == 13
                #                     or send.DIOValue == 11 or send.DIOValue == 15 or send.DIOValue == 24 or send.DIOValue == 27 or send.DIOValue == 31):
                if step != 'begin' and send.Web == False:  # gazebo test
                    motor.bodyauto_close(0)
                    print("bodyauto_close",motor.bodyauto_close)#!!!!!!!!!!
                    target = target_location()
                    motor = motor_move()
                    step = 'begin'
                    send.sendHeadMotor(1, 2048, 30)  # reset head
                    send.sendHeadMotor(2, 2048, 30)
                    time.sleep(0.05)
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
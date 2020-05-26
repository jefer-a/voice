#!/usr/bin/env python
# -*- coding:utf-8 -*-

import roslib; roslib.load_manifest('voice')
import os, sys
import playsound
import random
import rospy
import math
import copy
import re

from geometry_msgs.msg import Twist #运动速度结构体类型
from std_msgs.msg import String

# Python2.5 初始化后会删除 sys.setdefaultencoding,重新载入 
reload(sys)
sys.setdefaultencoding('utf-8')

class cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # 初始化默认速度
        self.speed_Linear = 0.2
        self.speed_Angular = 0.5
        self.msg = Twist()
        # self.tts="好的"

        # 震荡参数及msg缓存类堆栈，控制发布时间
        self.oscillation = {'oscil_Flag':0,'loop_Flag':0,'msg_Num':0,'keyWord':1,1:{'oscil_Num':0,'msg_Buff':Twist()}}
        self.oscillation_backup={}
        self.oscil_rate=10

        # 发布与订阅
        # turtlesim_mode
        # self.pub_ = rospy.Publisher('/turtle1/cmd_vel', Twist,queue_size=1)
        # turtlebot3_burger
        self.pub_ = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
        # self.pubtts_ = rospy.Publisher('voice_feedback',String,queue_size=10)
        rospy.Subscriber('voice_txt', String, self.cmd_Extraction)

        # 预加载部分
        self.mp3_path=os.path.dirname(__file__)
        self.digit={"一":1,"二":2,"两":2,"三":3,"四":4,"五":5,"六":6,"七":7,"八":8,"九":9,"十":10}

        # 编译正则表达式
        # self.pattern1=re.compile(r'(?P<router>圆|正方形|长方形|矩形|前进|向前|加速|减速|左转|右转|后退|停止|立定|继续|恢复)')
        self.pattern1=re.compile(ur'(?P<router>\u5706|\u6B63\u65B9\u5F62|\u957F\u65B9\u5F62|\u77E9\u5F62|\u524D\u8FDB|\u5411\u524D|\u52A0\u901F|\u51CF\u901F|\u5DE6\u8F6C|\u53F3\u8F6C|\u540E\u9000|\u505C\u6B62|\u7ACB\u5B9A|\u7EE7\u7EED|\u6062\u590D)')
        # self.pattern2=re.compile(r'(?P<way>半径|直径|边长|前进|向前|后退)[^\d|一|二|三|四|五|六|七|八|九|十](?P<length>\d+|一|二|三|四|五|六|七|八|九|十).*(?P<direction>正|顺|逆)*.*(?P<router>圆|正方形)')
        self.pattern2=re.compile(ur'(?P<way>\u534A\u5F84|\u76F4\u5F84|\u8FB9\u957F|\u524D\u8FDB|\u5411\u524D|\u540E\u9000)\D*(?P<length>\d+|\u4E00|\u4E8C|\u4E09|\u56DB|\u4E94|\u516D|\u4E03|\u516B|\u4E5D|\u5341)[^\u6B63|\u987A|\u9006]*(?P<direction>\u6B63(?:.[^\u5F62])|\u987A|\u9006)?')
        # 长-宽 类型长方形匹配
        self.pattern3=re.compile(ur'(?P<way_length>\u957F)\D*(?P<length>\d+|\u4E00|\u4E8C|\u4E09|\u56DB|\u4E94|\u516D|\u4E03|\u516B|\u4E5D|\u5341).*(?P<way_wide>\u5BBD)\D*(?P<wide>\d+|\u4E00|\u4E8C|\u4E09|\u56DB|\u4E94|\u516D|\u4E03|\u516B|\u4E5D|\u5341)[^\u6B63|\u987A|\u9006]*(?P<direction>\u6B63|\u987A|\u9006)?')
        # 宽-长 类型长方形匹配
        self.pattern4=re.compile(ur'(?P<way_wide>\u5BBD)\D*(?P<wide>\d+|\u4E00|\u4E8C|\u4E09|\u56DB|\u4E94|\u516D|\u4E03|\u516B|\u4E5D|\u5341).*(?P<way_length>\u957F)\D*(?P<length>\d+|\u4E00|\u4E8C|\u4E09|\u56DB|\u4E94|\u516D|\u4E03|\u516B|\u4E5D|\u5341)[^\u6B63|\u987A|\u9006]*(?P<direction>\u6B63|\u987A|\u9006)?')
        # 循环指令匹配pattern5=re.compile(r'(?P<notLoop>一个|走个)')
        self.pattern5=re.compile(ur'(?P<notLoop>\u8D70\u4E2A|\u4E00\u4E2A)')

        # 不同路径类型Twist发布频率控制
    	rate = rospy.Rate(self.oscil_rate)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)
            if self.oscillation['oscil_Flag']==1:
                key=self.oscillation['keyWord']
                self.msg=self.oscillation[key]['msg_Buff']
                self.oscillation[key]['oscil_Num']-=1
                if self.oscillation[key]['oscil_Num']==0:
                    self.oscillation['msg_Num']-=1
                    self.oscillation['keyWord']+=1
                    if self.oscillation['msg_Num']==0 and self.oscillation['loop_Flag']==1:
                        self.oscillation=copy.deepcopy(self.oscillation_backup)
                    elif self.oscillation['msg_Num']==0 and self.oscillation['loop_Flag']==0:
                        self.oscillation['oscil_Flag']=2
                        # random_mp3=random.choice([1,2,3,4])
                        # mp3_file=self.mp3_path+'/end'+str(random_mp3)+'.mp3'
                        # playsound.playsound(mp3_file)
            elif self.oscillation['oscil_Flag']==2:
                self.msg=Twist()
                self.oscillation['oscil_Flag']=0
            rate.sleep()

    #指令提取函数
    def cmd_Extraction(self, msg):
        rospy.loginfo(msg.data)
        #订阅数据字符串预处理
        command_Str=msg.data
        if '关闭' in command_Str:
            rospy.signal_shutdown("关闭机器人")

        # 尝试匹配运动路径控制关键词
        try:
            regex1=self.pattern1.search(command_Str.decode('utf-8'))
            router=regex1.group('router')
        except:
            # 非有效路径控制指令，退出回调函数
            return
        # print(router)

        # 圆形路径
        if router==u'\u5706':
            # print("flagtype=圆形")
            try:
                regex2=self.pattern2.search(command_Str.decode('utf-8'))
                way=regex2.group('way')
                length=regex2.group('length')
                # print(way)
                # print(length)
            except:
                return
            length=float(self.digit.get(length.encode('utf-8'),length))
            direction=self.direc(regex2)
            # print(direction)
            if way==u'\u534A\u5F84':
                radius=length
            elif way==u'\u76F4\u5F84':
                radius=length/2
            else:
                radius=0
            # print(radius)
            if radius:
                oscil_Buff=self.msg_Twist(2*radius*math.pi,1)
                self.oscillation[1]=oscil_Buff.copy()
                self.oscillation[1]['msg_Buff'].angular.z\
                 =self.oscillation[1]['msg_Buff'].linear.x/radius*direction
                self.oscillation['msg_Num']=1
                self.notLoop(command_Str)
                
        # 正方形
        elif router==u'\u6B63\u65B9\u5F62':
            # print("flagtype=正方形")
            try:
                regex2=self.pattern2.search(command_Str.decode('utf-8'))
                way=regex2.group('way')
                length=regex2.group('length')
                # print(way)
                # print(length)
            except:
                return
            length=float(self.digit.get(length.encode('utf-8'),length))
            direction=self.direc(regex2)
            # print(direction)
            if way==u'\u8FB9\u957F':
                length_buff=self.msg_Twist(length,1)
                angular_buff=self.msg_Twist(0.5*math.pi,2,direction)
                self.oscillation['msg_Num']=8
                for i in range(1,9,2):
                    self.oscillation[i]=length_buff.copy()
                    self.oscillation[i+1]=angular_buff.copy()
                self.notLoop(command_Str)

        # 长方形 矩形
        elif router in (u'\u957F\u65B9\u5F62', u'\u77E9\u5F62'):
            # print("flagtype=长方形")
            try:
                regex3=self.pattern3.search(command_Str.decode('utf-8'))
                length=regex3.group('length')
                wide=regex3.group('wide')
                # print(length)
                # print(wide)
            except:
                try:
                    regex3=self.pattern4.search(command_Str.decode('utf-8'))
                    length=regex3.group('length')
                    wide=regex3.group('wide')
                    # print(length)
                    # print(wide)
                except:
                    return
            length=float(self.digit.get(length.encode('utf-8'),length))
            wide=float(self.digit.get(wide.encode('utf-8'),wide))
            direction=self.direc(regex3)
            if length and wide:
                length_buff=self.msg_Twist(length,1)
                wide_Buff=self.msg_Twist(wide,1)
                angular_buff=self.msg_Twist(0.5*math.pi,2,direction)
                self.oscillation['msg_Num']=8
                for i in range(1,9,4):
                    self.oscillation[i]=length_buff.copy()
                    self.oscillation[i+1]=angular_buff.copy()
                    self.oscillation[i+2]=wide_Buff.copy()
                    self.oscillation[i+3]=angular_buff.copy()
                self.notLoop(command_Str)

        # 前进 向前
        elif router in (u'\u524D\u8FDB',u'\u5411\u524D'):
            # print("flagtype=直线")
            try:
                regex4=self.pattern2.search(command_Str.decode('utf-8'))
                way=regex4.group('way')
                length=regex4.group('length')
                # print(way)
                # print(length)
                length=float(self.digit.get(length.encode('utf-8'),length))
                if way in (u'\u524D\u8FDB',u'\u5411\u524D'):
                    length_buff=self.msg_Twist(length,1)
                    self.oscillation[1]=length_buff.copy()
                    self.oscillation['msg_Num']=1
                    self.oscillation['oscil_Flag']=1
            except:
                # print("前进3")
                self.msg.linear.x = self.speed_Linear
                self.msg.angular.z = 0
                self.oscillation['oscil_Flag']=0

        # 加速
    	elif router==u'\u52A0\u901F':
            # print("加速11")
            if self.oscillation['oscil_Flag']!=1 and\
             self.msg.linear.x>0 and self.msg.linear.x*2<=2.5:
                if self.msg.linear.z!=0:
                    self.msg.linear.x = self.msg.linear.x*2
                    self.msg.angular.z = self.msg.angular.z*2
                else:
                    self.msg.linear.x +=0.2

        # 减速
        elif router==u'\u51CF\u901F':
            # print("减速2")
            if self.oscillation['oscil_Flag']!=1 and self.msg.linear.x-0.2>0:
                if self.msg.linear.z!=0:
                    self.msg.linear.x = self.msg.linear.x/2
                    self.msg.angular.z = self.msg.angular.z/2
                else:
                    self.msg.linear.x -= 0.2

        # 左转
        elif router==u'\u5DE6\u8F6C':
            # print("左转4")
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed_Angular:
                    self.msg.angular.z += 0.1
                    if self.msg.angular.z > self.speed_Angular:
                        self.msg.angular.z = self.speed_Angular
                    self.oscillation['oscil_Flag']=0
            else:
                self.msg.angular.z = self.speed_Angular
                self.oscillation['oscil_Flag']=0

        # 右转
        elif router==u'\u53F3\u8F6C':   
            # print("右转5")
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed_Angular:
                    self.msg.angular.z -= 0.1
                    if self.msg.angular.z < -self.speed_Angular:
                        self.msg.angular.z = -self.speed_Angular
                    self.oscillation['oscil_Flag']=0
            else:
                self.msg.angular.z = -self.speed_Angular
                self.oscillation['oscil_Flag']=0

        # 后退
        elif router==u'\u540E\u9000':
            # print("后退6")
            try:
                regex4=self.pattern2.search(command_Str.decode('utf-8'))
                way=regex4.group('way')
                length=regex4.group('length')
                # print(way)
                # print(length)
                length=float(self.digit.get(length.encode('utf-8'),length))
                if way==u'\u540E\u9000':
                    length_buff=self.msg_Twist(length,1,-1)
                    self.oscillation[1]=length_buff.copy()
                    self.oscillation['msg_Num']=1
                    self.oscillation['oscil_Flag']=1
            except:
                self.msg.linear.x = -self.speed_Linear
                self.msg.angular.z = 0
                self.oscillation['oscil_Flag']=0

        # 停止 立定
        elif router in (u'\u505C\u6B62',u'\u7ACB\u5B9A'):
            # print("停止7")
            self.oscillation['oscil_Flag']=2

        # 继续 恢复
        elif router in (u'\u7EE7\u7EED',u'\u6062\u590D'):
            # print("继续")
            self.oscillation['oscil_Flag']=1


        # self.pubtts_.publish(self.tts)
        # random_mp3=random.choice([1,2,3])
        # mp3_file=self.mp3_path+'/ok'+str(random_mp3)+'.mp3'
        # playsound.playsound(mp3_file)
        # print(mp3_file)
        # os.system("play "+mp3_file)
        print("处理完成")


    # stop the robot!
    def cleanup(self):
        twist = Twist()
        self.pub_.publish(twist)

    # 尝试匹配路径轨迹方向
    def direc(self,regex):
        try:
            direction=regex.group('direction')[0:1]
            # print(direction)
            if direction in (u'\u6B63',u'\u987A'):
                return -1
            else:
                return 1
        except:
            return 1

    # Twist消息类型转换处理
    def msg_Twist(self,value,type,direction=1):
        self.oscillation = {'oscil_Flag':0,'msg_Num':0,'keyWord':1,'loop_Flag':0}
        temp={'oscil_Num':0,'msg_Buff':Twist()}
        if type==1:
            temp['oscil_Num']=(self.oscil_rate*value)//self.speed_Linear
            speed_temp=(self.oscil_rate*value)/temp['oscil_Num']
            temp['msg_Buff'].linear.x=speed_temp*direction
        elif type==2:
            temp['oscil_Num']=(self.oscil_rate* value)//self.speed_Angular
            speed_temp=(self.oscil_rate*value)/temp['oscil_Num']
            temp['msg_Buff'].angular.z=speed_temp*direction
        return temp

    # 循环指令匹配
    def notLoop(self,command_Str,flag=1):
        regex_notloop=self.pattern5.search(command_Str.decode('utf-8'))
        if regex_notloop:
            self.oscillation['loop_Flag']=0
        else:
            self.oscillation['loop_Flag']=1
        self.oscillation_backup=copy.deepcopy(self.oscillation)
        self.oscillation['oscil_Flag']=flag
        self.oscillation_backup['oscil_Flag']=flag


if __name__=="__main__":
    rospy.init_node('voice_cmd_node')
    try:
        cmd_vel()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
# -*- coding:utf-8 -*-
#speech_script.py

import os,sys
import time

def select(txtFile):
    while True:
        print("请选择发布模式：")
        print("1.自动延时逐条发布")
        print("2.手动延迟逐条发布")
        print("3.手动选择单条发布")
        option=input()
        if  '1' in option:
            publisher_one(txtFile)
        elif '2' in option:
            publisher_two(txtFile)
        elif '3' in option:
            publisher_three(txtFile)
        else:
            print("错误输入，请再次输入！")
            continue
        print("是否继续尝试其他模式？[y/n]:")
        option=input()
        if 'y' in option:
            continue
        else:
            break

def publisher_one(txtFile):
    for commod in txtFile.readlines():
        if commod[0]=='#':
            continue
        cmd_pub(commod,15)

def publisher_two(txtFile):
    for commod in txtFile.readlines():
        cmd_pub(commod)
        print("回车发布下一条指令！")
        input()

def publisher_three(txtFile):
    i=0
    commods=txtFile.readlines()
    print("请选择路径控制指令编号(退出输入[n]):")
    for commod in commods:
        i+=1
        print(str(i)+"："+commod)
    while True:
        select_commod=input()
        try:
            if 0 < int(select_commod) <= i:
                commod=commods[int(select_commod)-1]
                cmd_pub(commod)
                print("请选择路径控制指令编号(退出输入[n]):")
                continue
        except :
        	pass
        try:
            if select_commod=='n':
                print("退出模式3!")
                break
        except:
            print('无效编号，请再次输入!')

def cmd_pub(commod,delay=1):
	print(commod)
	cmd="rostopic pub -1 /voice_txt std_msgs/String "+"\""+commod+"\""
	os.system("gnome-terminal -e "+"\'"+cmd+"\'")
	time.sleep(delay)



if __name__ == "__main__":
    try:
        script_path=os.path.dirname(__file__)
        txt_path=script_path+'/speech.txt'
        txtFile=open(txt_path,'r',encoding="utf-8")
        select(txtFile)
    except:
        print("speech.txt文件不存在")
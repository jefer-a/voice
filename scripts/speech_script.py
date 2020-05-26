#!/usr/bin/env python3
# -*- coding:utf-8 -*-
#speech_script.py

import os,sys
import time

def select():
    script_path=os.path.dirname(__file__)
    txt_path=script_path+'/speech.txt'
    while True:
        txtFile=open(txt_path,'r',encoding="utf-8")
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
            txtFile.close()
            return

def publisher_one(txtFile):
    for command in txtFile.readlines():
        if command[0]=='#':
            continue
        cmd_pub(command,15)

def publisher_two(txtFile):
    for command in txtFile.readlines():
        cmd_pub(command)
        print("回车发布下一条指令！")
        input()

def publisher_three(txtFile):
    i=0
    commands=txtFile.readlines()
    print("请选择路径控制指令编号(退出输入[n]):")
    for command in commands:
        i+=1
        print(str(i)+"："+command)
    while True:
        seleant_commod=input()
        try:
            if 0 < int(seleant_commod) <= i:
                command=commands[int(seleant_commod)-1]
                cmd_pub(command)
                print("请选择路径控制指令编号(退出输入[n]):")
                continue
        except :
        	pass
        try:
            if seleant_commod=='n':
                print("退出模式3!")
                return
        except:
            print('无效编号，请再次输入!')

def cmd_pub(command,delay=1):
	print("Command:",command)
	cmd="rostopic pub -1 /voice_txt std_msgs/String "+"\""+command+"\""
	os.system("gnome-terminal -e "+"\'"+cmd+"\'")
	time.sleep(delay)



if __name__ == "__main__":
    try:
        select()
    except:
        print("程序异常退出/speech.txt文件不存在")
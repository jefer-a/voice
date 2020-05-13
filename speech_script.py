#!/usr/bin/env python3
# -*- coding:utf-8 -*-
#speech_script.py

import os,sys
import time

def select():
    while True:
        print("请选择发布模式：")
        print("1.自动延时逐条发布")
        print("2.手动延迟逐条发布")
        print("3.手动选择单条发布")
        txtFile=open("speech.txt",'r',encoding="utf-8")
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
        os.system("rostopic pub -1 /voice_txt /sdt_message/string "+commod)
        time.sleep(15)
        print(commod)

def publisher_two(txtFile):
    for commod in txtFile.readlines():
        # print(commod)
        if commod[0]=='#':
            continue
        os.system("rostopic pub -1 /voice_txt /sdt_message/string "+commod)
        time.sleep(1)
        print("发布下一条指令请回车！")
        input()

def publisher_three(txtFile):
    i=0
    count=0
    commods=txtFile.readlines()
    for commod in commods:
        i+=1
        print(str(i)+"："+commod)
    print("请选择欲发布的路径控制指令编号(退出输入[n]):")
    while True:
        count+=1
        if count >10:
            for commod in txtFile.readlines():
                i+=1
                print(str(i)+"："+commod)
            count=0
        select_commod=input()
        try:
            if 0 < int(select_commod) <= i:
                commod=commods[int(select_commod)-1]
                os.system("rostopic pub -1 /voice_txt /sdt_message/string "+commod)
                time.sleep(1)
        except :
            if select_commod=='n':
                print("退出模式3!")
                break
            else:
                print('无效输入，请再次输入!')
                continue
        print("请选择欲发布的路径控制指令编号(退出输入[n]):")



if __name__ == "__main__":
    if os.path.isfile("speech.txt"):
        select()
    else:
        print("speech.txt文件不存在")
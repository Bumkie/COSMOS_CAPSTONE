#!/usr/bin/env python3

"""
Node for control PCA9685 using AckermannDriveStamped msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

#완성
import threading
import socket
from COSMOS.Plan.Planner import Planner
from COSMOS.Test.TelloVirtualController import TelloVirtualController
from COSMOS.DonkeyCar.DonkeyCar import DonkeyCar
import rospy



class Main:
    
    def __init__(self):

        #종료를 위한 stop_event
        self.stop_event = threading.Event()

        rospy.init_node("donkey_control") # "donkey_control"라는 이름으로 ROS 노드를 초기화

        self.donkeycar = DonkeyCar(self,"donkey_ros") # donkeycar 객체 생성
        
        #Tello의 주소, 포트
        self.tello_address = ('192.168.10.1',8889) #텔로에게 접속했을 때, 텔로의 IP주소

        #포트가 값을 읽어오는 시간(초)
        self.interval8889 = 10

        #소켓 선언
        self.socket8889 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP 통신 소켓 객체를 생성(command용)
        self.socket8889.bind(('', 8889)) #소켓 객체를 텔로와 바인딩(8889 포트)
        self.socket8889.settimeout(self.interval8889)  # 10초 시간 제한 설정

        #연결 확인
        print(">>> [Robomaster Tello Talent] 연결 대기중...")
        
        self.socket8889.sendto("command".encode('utf-8'), self.tello_address)
        response,addr= self.socket8889.recvfrom(1024)
        print(">>> [Robomaster Tello Talent] 8889 포트 연결: {}".format(addr))
        
        self.socket8889.sendto("streamon".encode('utf-8'), self.tello_address)
        response,addr = self.socket8889.recvfrom(1024)
        print(">>> [Robomaster Tello Talent] 비디오스트림 On: {} {}".format(response.decode('utf-8'),addr))
        
        self.socket8889.sendto("motoron".encode('utf-8'), self.tello_address)
        response,addr = self.socket8889.recvfrom(1024)
        print(">>> [Robomaster Tello Talent] 모터 On: {} {}".format(response.decode('utf-8'),addr))
        
        self.socket8889.sendto("downvision 1".encode('utf-8'), self.tello_address)
        response,addr = self.socket8889.recvfrom(1024)
        print(">>> [Robomaster Tello Talent] 다운비전 On: {} {}".format(response.decode('utf-8'),addr))


        self.socket8889.sendto("battery?".encode('utf-8'), self.tello_address)
        response,addr = self.socket8889.recvfrom(1024)
        print(f">>> battery: {response}")
        if int(response) < 25:
            print("ERROR: battery lack..")
            exit(0)
        print(">>> [Robomaster Tello Talent] 연결 완료")

        
        #객체 생성
        self.planner = Planner(self,self.donkeycar)
        
        self.virtual_controller = TelloVirtualController(self)
        
        #GUI 메인 루프 시작
        print(">>> 프로그램 실행")
        self.virtual_controller.root.mainloop()



if __name__ == "__main__":
    print(">>> 프로그램 준비중...")
    Main()


    
    






        

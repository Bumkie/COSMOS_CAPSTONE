#완성
import threading
import sys
import traceback
from time import sleep
from COSMOS.ObjectDetector.YOLOv5 import YOLOv5
from numpy import *
import cv2
import time
import socket
import math

class Planner:
    """
    연산을 담당하는 클래스
    1) 거리 정보가 안전 거리 이내인 경우, 윈도우 정보(존재하는 경우)를 가져와서 물체의 실제 크기를 계산
    2) 계산한 크기를 바탕으로 물체의 실제 크기에 대한 좌표값을 생성
    3) 생성한 좌표값을 바탕으로 회피명령을 생성
    4) 회피명령을 queue에 저장
    """
    
    
    
    #=====Planner의 인스턴스를 생성시 실행될 함수=====
    def __init__(self, main, donkey):
        
        #main
        self.main = main  
        self.stop_event = main.stop_event
        self.socket8889 = main.socket8889
        self.tello_address = main.tello_address
        self.donkey = donkey
   
        #각 센서가 저장하는 값
        self.info_8889Sensor_cmd = None #수행확인명령
        self.info_11111Sensor_frame = None #Frame
        self.info_11111Sensor_image = None
        self.info_11111Sensor_coor = None
        
        #객체감지를 위한 YOLOv5 객체
        self.YOLOv5 = YOLOv5()

        #tello 영상의 크기
        self.frame_height = 320
        self.frame_width = 240
        self.target_center = (self.frame_width // 2, self.frame_height // 2)
        
        #착륙장의 크기
        self.person_h = 270 #mm
        self.person_w = 180 #mm
        self.person_r = self.person_h / self.person_w
        self.allowable_error = 0.5 #착륙장을 인식할 때, 가로/세로비로 인식할 예정 => 이 때 비율에 대한 허용오차
        
        #저장해야 할 좌표들
        self.tv_coor = None #((x1,y1), (x2,y2))
        self.person_coor = None #((x1,y1), (x2,y2))
        self.banana_coor = None #((x1,y1), (x2,y2))
        
        #safe_landing: 착륙장을 인식하여 착륙하는 것으로, 중복실행을 방지하기 위해 토글 제어
        self.btn_safe_lading = False
        self.btn_mission = False
        
        #인식할 바닥 가로폭
        self.c_len = 100 #1m 인식
        self.correct_h = (self.c_len*228)/self.frame_width #올라가야할 높이
        
        #목표물의 좌표
        self.real_coor = None
        
        #스레드 실행
        self.thr_8889 = threading.Thread(target=self.func_8889, daemon=True)
        self.thr_8889.start()
        
        self.thr_yolo = threading.Thread(target=self.func_yolo, daemon=True)
        self.thr_yolo.start()
        
        self.thr_11111 = threading.Thread(target=self.func_11111, daemon=True)
        self.thr_11111.start()
        

        # self.thr_mission = threading.Thread(target=self.func_mission, daemon=True)

                
    #=====스레드에서 실행될 함수=====
    #메인 스레드
    def func_yolo(self):

        while not self.stop_event.is_set():
            
            # 프레임을 받아온다
            frame = self.get_info_11111Sensor_frame()
            
            if frame is None or frame.size == 0:
                continue
            
            # 프레임을 YOLO에 보낸다
            image, object_coor = self.YOLOv5.detect_from_frame(frame)
        
            # 전달받은 값들을 저장한다
            self.set_info_11111Sensor_image(image)
            self.set_info_11111Sensor_coor(object_coor)

            cnt1, cnt2, cnt3 =0,0,0

            for detection_info in object_coor:
                coor1, coor2, name = detection_info
                
                
                #인식한 대상이 사람 혹은 tv인 경우만 인식
                if not(name == 'person' or name == 'tv' or name == 'keyboard'):
                    continue
                
                object_w = coor2[0]-coor1[0]
                object_h = coor2[1]-coor1[1]
                object_r = object_h/object_w
                
                condition1 = (name == 'person' and abs(object_r - self.person_r) <= self.allowable_error)
                condition2 = (name == 'tv' and abs(object_r - 1) <= self.allowable_error)
                condition3 = (name == "keyboard")
                
                
                
                # person 혹은 tv를 인식한 경우, 좌표를 저장
                if condition1:
                    self.person_coor = (coor1,coor2)
                    cnt1+= 1
                
                elif condition2:
                    self.tv_coor = (coor1,coor2)
                    cnt2+= 1
                    
                elif condition3:
                    self.banana_coor = (coor1,coor2)
                    cnt3+= 1
            
            if cnt1 == 0:
                self.person_coor = None
            if cnt2 == 0:
                self.tv_coor = None
            if cnt3 == 0:
                self.banana_coor = None
    
     
    
    #frame을 받아오는 스레드
    def func_11111(self):
        
        cap = cv2.VideoCapture("udp://"+self.tello_address[0]+":"+"11111")
        
        while not self.stop_event.is_set():
            ret, frame = cap.read()
            
            if frame is not None:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                self.set_info_11111Sensor_frame(frame)
            cv2.waitKey(3)


            
    def func_8889(self):
        cnt = 0
        while not self.stop_event.is_set():
            try:
                self.data_8889 = self.socket8889.recv(1024).decode('utf-8')
                print(f">>> [func_8889] recv: {self.data_8889}")           


            except socket.timeout:
                # print(f">>> [func_8889] {self.interval8889}초 동안 데이터를 수신하지 못했습니다")
                self.socket8889.sendto("command".encode('utf-8'), self.tello_address)

        
        self.socket8889.close()
    
    
    def func_mission(self):
        
        self.socket8889.sendto('takeoff'.encode('utf-8'), self.tello_address) 
        time.sleep(5)
        #1. PID 제어를 통해 기준 높이까지 상승
        err_cnt = 0
        while not self.pid_control2(0.2) and not self.stop_event.is_set():
            err_cnt+=1
            if err_cnt > 0:
                print(">>> [Mission] 착륙장 인식 실패")
                self.btn_mission = False 
                return
        print(">>> [Mission] 높이 제어 성공")
        if self.stop_event.is_set():
            self.btn_mission = False 
            return 
        
        #2. 전방 이동
        self.socket8889.sendto('go 100 0 0 50'.encode('utf-8'), self.tello_address)   
        print(">>> [Mission] 전방 이동 성공")
        time.sleep(1)
        
        #3. 목표물 탐색
        err_cnt = -1
        obj_coor = self.search()
        
        while obj_coor is None and not self.stop_event.is_set():
            err_cnt+=1
            if err_cnt > 3:
                print(">>> [Mission] 목표물 인식 실패")
                break
            
            obj_coor = self.search()
        
        if self.stop_event.is_set():
            self.btn_mission = False 
            return 
        
        if obj_coor is None:
            #복귀
            self.socket8889.sendto('go -110 0 0 50'.encode('utf-8'), self.tello_address)   
            time.sleep(5)
            print(">>> [Mission] 후방 이동 성공")
            self.main.virtual_controller.safe_land()

            while self.btn_safe_lading:
                time.sleep(3)

            print(">>> [Mission] return success")
            self.btn_mission = False
            return
        
        #4-2. 좌표 계산
        print(">>> [Mission] 목표물 인식 성공")
        def cal_coor(obj_coor):
            x1,y1 = obj_coor[0]
            x2,y2 = obj_coor[1]
            
            x_center = (x1+x2)//2
            y_center = (y1+y2)//2
            
            x_offset = self.frame_width // 2
            y_offset = self.frame_height // 2
            
            #영상 중심의 좌표계
            x_new = (x_center - x_offset)*0.42
            y_new = (y_offset - y_center)*0.42
            
            #차량을 중심으로 하는 좌표계
            x_real = x_new
            y_real = 100+y_new
            
            return (x_real,y_real)

        
        self.real_coor = cal_coor(obj_coor)
        print(f">>> [Mission] 목표물 좌표: {self.real_coor}")
        
        #복귀
        time.sleep(3)
        print(">>> [Mission] back")
        self.socket8889.sendto('go -100 0 0 50'.encode('utf-8'), self.tello_address)   
        time.sleep(7)
        print(">>> [Mission] 후방 이동 성공")
        self.main.virtual_controller.safe_land()

        while self.btn_safe_lading:
            time.sleep(3)

        print(">>> [Mission] return success")
        
        time.sleep(4)
        
        self.autodrive()

        self.btn_mission = False
        
        

    
    
    def safe_land(self):
        # self.socket8889.sendto(msg.encode('utf-8'), self.tello_address)s
        # print("debug:",name, abs(object_r- self.person_r), object_w, object_h)
        
        if self.person_coor is None and self.tv_coor is None:
            print(">>> [Robomaster Tello Talent] 착륙장 인식 실패")
            self.btn_safe_lading = False 
            return
        
        #PID 제어(하강할 속도, 인식에 대한 허용 오차)
        is_success = self.pid_control(-30, 0.2)
        
        #착륙장을 인식하여 하강한 경우
        if is_success:
            self.socket8889.sendto('stop'.encode('utf-8'), self.tello_address)
            time.sleep(1)
            self.socket8889.sendto('land'.encode('utf-8'), self.tello_address)     
            print(">>> [Robomaster Tello Talent] 착륙 성공")
        
        #착륙장 인식에 실패한 경우
        else:
            self.socket8889.sendto('rc 0 0 0 0'.encode('utf-8'), self.tello_address)
            print(">>> [Robomaster Tello Talent] 인식 실패")

        self.btn_safe_lading = False   



    def pid_control(self, down_op, error_rate):
        
        x_cnt = 0 #잘못 인식한 횟수
        o_cnt = 0 #오차범위 내로 인식한 횟수
        
        pid_x = PID() #가로에 대한 PID
        pid_y = PID() #세로에 대한 PID
        
        while not self.stop_event.is_set():
            
            #인식에 실패하면
            person_coor = self.person_coor
            tv_coor = self.tv_coor
            
            if person_coor is None and tv_coor is None:
                x_cnt += 1
                
                #정지 명령 전송
                self.socket8889.sendto('rc 0 0 0 0'.encode('utf-8'), self.tello_address)
                
                # 잘못 인식한 횟수가 6번 이상이면
                if x_cnt > 15:
                    print(">>> [Robomaster Tello Talent] person 인식에 15회 이상 실패했습니다.")
                    
                    #정지 명령 전송
                    self.socket8889.sendto('rc 0 0 0 0'.encode('utf-8'), self.tello_address)
                    
                    return False
                
                # 0.5초 대기 후, 다시 시도
                time.sleep(0.5)
                continue
            
            #인식에 성공하면 잘못 인식한 횟수 초기화
            x_cnt = 0
            
            x1,x2,y1,y2 = 0,0,0,0
            
            if tv_coor:
                x1, y1 = tv_coor[0]
                x2, y2 = tv_coor[1]
            
            if person_coor:
                x1, y1 = person_coor[0]
                x2, y2 = person_coor[1]
            
            # 인식한 대상의 중심점 계산
            bbox_center = ((x1 + x2) // 2, (y1 + y2) // 2)
            
            # 인식한 대상과 영상 중심의 차이값
            error_x = self.target_center[0] - bbox_center[0]
            error_y = self.target_center[1] - bbox_center[1]
            print(f"[debug] error_x: {error_x}, error_y: {error_y}")
            
            #PID 값 갱신
            pid_x.update(error_x)
            pid_y.update(error_y)
            
            #PID제어를 통한 보정값
            adjust_x = pid_x.output
            adjust_y = pid_y.output
            
            #보정값을 바탕으로 제어 명령 생성
            cmd_x = int(adjust_x)*(6-o_cnt) 
            if abs(cmd_x) < 20 and cmd_x!=0:
                cmd_x = cmd_x/abs(cmd_x) * 20
            
            cmd_y = int(adjust_y)*(-6+o_cnt) 
            if abs(cmd_y) < 20 and cmd_y!=0:
                cmd_y = cmd_y/abs(cmd_y) * 20

            cmd = f'rc {cmd_x} {cmd_y} 0 0'
            print(f"[debug] 제어명령: {cmd}\n")
            
            #제어명령 전송
            self.socket8889.sendto(cmd.encode('utf-8'), self.tello_address)
            time.sleep(0.3)
            self.socket8889.sendto('rc 0 0 0 0'.encode('utf-8'), self.tello_address)
            time.sleep(1)
            
            #인식한 대상의 크기에 따른 허용오차율 계산
            error_for_x = (x2-x1)*error_rate
            error_for_y = (y2-y1)*error_rate
            
            #허용오차율 이내로 위치한 경우
            if abs(error_x) < error_for_x and abs(error_y) < error_for_y:
                o_cnt += 1
                print(f">>> [Robomaster Tello Talent] 위치 보정 완료: {o_cnt}")
                self.socket8889.sendto(f'rc 0 0 {down_op} 0'.encode('utf-8'), self.tello_address)
                time.sleep(0.5)
                
                if (person_coor and abs(x1-x2)>=120 and abs(y2-y1)>=180) or o_cnt == 5:
                    return True
            
            time.sleep(0.1)
        


    def pid_control2(self, error_rate):
        
        x_cnt = 0 #잘못 인식한 횟수
        o_cnt = 0 #오차범위 내로 인식한 횟수
        
        pid_x = PID() #가로에 대한 PID
        pid_y = PID() #세로에 대한 PID
        pid_h = PID() #높이에 대한 PID
        
        while not self.stop_event.is_set():
            
            #인식에 실패하면
            person_coor = self.person_coor
            tv_coor = self.tv_coor
            
            if person_coor is None and tv_coor is None:
                x_cnt += 1
                
                #짝수일 때는 하강
                if x_cnt%2 == 0:
                    self.socket8889.sendto('go 0 0 -20 30'.encode('utf-8'), self.tello_address)
                    time.sleep(1)
                
                #홀수일 때는 상승
                else:
                    self.socket8889.sendto('go 0 0 20 30'.encode('utf-8'), self.tello_address)
                    time.sleep(1)
                    
                
                # 잘못 인식한 횟수가 10번 이상이면
                if x_cnt > 15:
                    print(">>> [Robomaster Tello Talent] 착륙장 인식에 15회 이상 실패했습니다.")
                    
                    #정지 명령 전송
                    self.socket8889.sendto('rc 0 0 0 0'.encode('utf-8'), self.tello_address)
                    
                    return False
                
                # 0.5초 대기 후, 다시 시도
                time.sleep(0.5)
                continue
            
            #인식에 성공하면 잘못 인식한 횟수 초기화
            x_cnt = 0
            
            x1,x2,y1,y2 = 0,0,0,0
            
            if tv_coor:
                x1, y1 = tv_coor[0]
                x2, y2 = tv_coor[1]
            
            if person_coor:
                x1, y1 = person_coor[0]
                x2, y2 = person_coor[1]
            
            #현재 높이 계산
            pixel_x = x2-x1
            pixel_y = y2-y1
            
            h = 0.5*((43*228)/pixel_x + (45.2*228)/pixel_y)
            if person_coor:
                h = 0.5*((18*228)/pixel_x + (27*228)/pixel_y)
             
            
            # 인식한 대상의 중심점 및 크기 계산
            bbox_center = ((x1 + x2) // 2, (y1 + y2) // 2)
            
            # 인식한 대상과 영상 중심의 차이값
            error_x = self.target_center[0] - bbox_center[0]
            error_y = self.target_center[1] - bbox_center[1]
            error_h = self.correct_h - h
            
            print(f"[debug] error_x: {error_x}, error_y: {error_y}, error_h: {error_h}")
            
            #PID 값 갱신
            pid_x.update(error_x)
            pid_y.update(error_y)
            pid_h.update(error_h)
            
            #PID제어를 통한 보정값
            adjust_x = pid_x.output
            adjust_y = pid_y.output
            adjust_h = pid_h.output
            
            #보정값을 바탕으로 제어 명령 생성
            cmd_x = int(adjust_x)*(6-o_cnt) 
            if abs(cmd_x) < 20 and cmd_x!=0:
                cmd_x = cmd_x/abs(cmd_x) * 20
            
            cmd_y = int(adjust_y)*(-6+o_cnt) 
            if abs(cmd_y) < 20 and cmd_y!=0:
                cmd_y = cmd_y/abs(cmd_y) * 20
            
            cmd_h = int((adjust_h)*(-6+o_cnt)*1) 
            if abs(cmd_h) < 20 and cmd_h!=0:
                cmd_h = cmd_h/abs(cmd_h) * 20

            cmd = f'rc {cmd_x} {cmd_y} {cmd_h} 0'
            print(f"[debug] 제어명령: {cmd}\n")
            
            #제어명령 전송
            self.socket8889.sendto(cmd.encode('utf-8'), self.tello_address)
            time.sleep(0.3)
            self.socket8889.sendto('rc 0 0 0 0'.encode('utf-8'), self.tello_address)
            time.sleep(1)
            
            #인식한 대상의 크기에 따른 허용오차율 계산
            error_for_x = pixel_x*error_rate
            error_for_y = pixel_y*error_rate
            error_for_z = h*error_rate
            
            #허용오차율 이내로 위치한 경우
            if abs(error_x) < error_for_x and abs(error_y) < error_for_y and abs(error_h)< error_for_z:
                o_cnt += 1
                print(f">>> [Robomaster Tello Talent] 위치 보정 완료: {o_cnt}")
                self.socket8889.sendto(f'rc 0 0 0 0'.encode('utf-8'), self.tello_address)
                time.sleep(0.5)
                
                if o_cnt == 5:
                    return True
            
            time.sleep(0.1)

    def search(self):
        print(">>> [Mission] search start")
        x_cnt = 0 #잘못 인식한 횟수
        move_cnt = 0

        while not self.stop_event.is_set():
            
            #인식에 실패하면
            banana_coor = self.banana_coor
            
            if banana_coor is None:
                x_cnt += 1
                
                #못 찾으면 하강
                if x_cnt%2 == 0:
                    move_cnt+= (-20)
                    cmd = 'go 0 0 -20 30'
                    self.socket8889.sendto(cmd.encode('utf-8'), self.tello_address)
                    time.sleep(1)
                    
                #홀수일 때는 상승
                else:
                    move_cnt+= (20)
                    cmd = 'go 0 0 20 30'
                    self.socket8889.sendto(cmd.encode('utf-8'), self.tello_address)
                    time.sleep(1)
                    
                # 잘못 인식한 횟수가 10번 이상이면
                if x_cnt > 10:
                    print(">>> [Mission] 목표물 인식에 10회 이상 실패했습니다.")
                    
                    #정지 명령 전송
                    self.socket8889.sendto('stop'.encode('utf-8'), self.tello_address)
                    
                    return None
                
                # 1초 대기 후, 다시 시도
                time.sleep(1)
                continue
        
            else:
                move_cnt = -move_cnt
                cmd = f"go 0 0 {move_cnt} 30"
                print(f">>> [search] {cmd}")
                self.socket8889.sendto(cmd.encode('utf-8'), self.tello_address)
                time.sleep(3)
                return banana_coor
            
            
    def autodrive(self):
        self.donkey.steering_servo.run(370)

        self.real_coor
        target_x = self.real_coor[0]
        target_y = self.real_coor[1]

        throttle_pulse = 380
        steering_pulse = 360

        NEUTRAL_STEERING_PULSE = 360 
        MAX_LEFT_STEERING_PULSE = 700
        MAX_RIGHT_STEERING_PULSE = 200


        current_x = 0
        current_y = 0
        current_rotation = 90

        angle = math.atan2((target_y - current_y), (target_x - current_x))
        
        steering_angle = math.degrees(angle)

        required_rotation = current_rotation - steering_angle

        steering_ratio = required_rotation / 140.0

        if required_rotation >0 :
            steering_pulse = int(NEUTRAL_STEERING_PULSE + steering_ratio * (MAX_RIGHT_STEERING_PULSE - NEUTRAL_STEERING_PULSE))

        elif required_rotation < 0 :
            steering_pulse = int(NEUTRAL_STEERING_PULSE - steering_ratio * (MAX_LEFT_STEERING_PULSE - NEUTRAL_STEERING_PULSE))

        else:
            steering_pulse = 360

        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

        move_time = 0.14 * distance/10

        self.donkey.steering_servo.run(360)


        while(move_time >0):

            self.donkey.steering_servo.run(steering_pulse)
            self.donkey.throttle.run(throttle_pulse)
            time.sleep(0.2)
            move_time -= 0.2
        
        self.donkey.throttle.run(0)
        self.donkey.steering_servo.run(370)
    #=====getter/setter 선언=====

       
    #11111Sensor_frame    
    def get_info_11111Sensor_frame(self):
        info = self.info_11111Sensor_frame
        return info
    
    def set_info_11111Sensor_frame(self, info):
        self.info_11111Sensor_frame = info
       
    #11111Sensor_image    
    def get_info_11111Sensor_image(self):
        info = self.info_11111Sensor_image
        return info
    
    def set_info_11111Sensor_image(self, info):
        self.info_11111Sensor_image = info
         
    #11111Sensor_coor
    def get_info_11111Sensor_coor(self):
        info = self.info_11111Sensor_coor
        return info
    
    def set_info_11111Sensor_coor(self, info):
        self.info_11111Sensor_coor = info



class PID:
    def __init__(self, P=0.1, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.last_error = 0
        self.SetPoint = 0 

        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        
        error = self.SetPoint - feedback_value
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

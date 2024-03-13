import tkinter
import threading
from PIL import ImageTk
from time import sleep
import traceback
from PIL import ImageTk
import RPi.GPIO as GPIO


class TelloVirtualController:
    """
    가상의 컨트롤러를 의미하는 클래스
    -GUI 화면을 띄움
    -Tello의 ToF값을 화면에 출력
    -YOLO의 감지화면을 화면에 출력
    -키보드 및 화면의 버튼을 통해 Tello를 조작
    -thread_stay_connection 스레드를 통해 지속적으로 Tello에게 "command" 메세지를 전달
    -종료시 stop_event를 실행
    """



    #=====VirtualController의 인스턴스를 생성시 실행될 함수=====
    def __init__(self, main):
        print(">>> [VirtualController] 생성")
        
        #main
        self.socket8889 = main.socket8889
        self.tello_address = main.tello_address
        self.stop_event = main.stop_event
        self.planner = main.planner

        #프로그램 종료를 위한 but_pin 정의
        self.but_pin = 13
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.but_pin, GPIO.IN)

        self.curr_value = GPIO.input(self.but_pin)
        self.prev_value = self.curr_value

        #Tello 조작 속도
        self.tello_speed = 50

        #화면 기본 설정
        self.root = tkinter.Tk()  # GUI 화면 객체 생성
        self.root.geometry("-10+0")
        self.root.wm_title("COSMOS TEST for RMTT") #GUI 화면의 title 설정  
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose) #종료버튼을 클릭시 실행할 함수 설정

        #화면에 띄울 문구 설정
        self.text_keyboard = tkinter.Label(self.root, justify="left", text="""
        W - Move Tello Up\t\t\tArrow Up - Move Tello Forward
        S - Move Tello Down\t\t\tArrow Down - Move Tello Backward
        A - Rotate Tello Counter-Clockwise\t\tArrow Left - Move Tello Left
        D - Rotate Tello Clockwise\t\tArrow Right - Move Tello Right
        """)
        self.text_keyboard.pack(side="top")

        #영상을 출력하기 위한 panel 선언
        self.panel_image = None

        #착륙 버튼
        self.btn_landing = tkinter.Button(self.root, text="Land", relief="raised", command=self.land)
        self.btn_landing.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)
        
        #보정 착륙 버튼
        self.btn_landing = tkinter.Button(self.root, text="Safe Land", relief="raised", command=self.safe_land)
        self.btn_landing.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)
        
        #이륙 버튼
        self.btn_takeoff = tkinter.Button(self.root, text="Takeoff", relief="raised", command=self.takeoff)
        self.btn_takeoff.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)
    
        #미션 버튼
        self.btn_mission = tkinter.Button(self.root, text="mission", relief="raised", command=self.mission)
        self.btn_mission.pack(side="bottom", fill="both", expand="yes", padx=10, pady=5)

        #키보드 버튼들과 Tello 동작을 바인딩
        self.keyboard_connection = tkinter.Frame(self.root, width=100, height=2)
        self.keyboard_connection.bind('<KeyPress-q>', self.on_keypress_q)
        self.keyboard_connection.bind('<KeyPress-w>', self.on_keypress_w)
        self.keyboard_connection.bind('<KeyPress-s>', self.on_keypress_s)
        self.keyboard_connection.bind('<KeyPress-a>', self.on_keypress_a)
        self.keyboard_connection.bind('<KeyPress-d>', self.on_keypress_d)
        self.keyboard_connection.bind('<KeyPress-Up>', self.on_keypress_up)
        self.keyboard_connection.bind('<KeyPress-Down>', self.on_keypress_down)
        self.keyboard_connection.bind('<KeyPress-Left>', self.on_keypress_left)
        self.keyboard_connection.bind('<KeyPress-Right>', self.on_keypress_right)
        self.keyboard_connection.pack(side="bottom")
        self.keyboard_connection.focus_set()

        #실행될 스레드 선언
        self.thread_print_video = threading.Thread(target=self.func_print_video, daemon=True)
        self.thread_print_video.start()

        self.thr_but = threading.Thread(target=self.func_but, daemon=True)
        self.thr_but.start()
    


    #=====버튼을 클릭했을 때 실행될 함수들=====
    def land(self): #return: Tello의 receive 'OK' or 'FALSE'
        self.send_cmd('land')

    def safe_land(self): #return: Tello의 receive 'OK' or 'FALSE'
        if self.planner.btn_safe_lading == False:
            self.planner.btn_safe_lading = True
            self.thr_safe_land = threading.Thread(target=self.planner.safe_land, daemon= True)
            self.thr_safe_land.start()
        else:
            print(">>> [VirtualController] safe land 수행 중..")

    def mission(self):
        if self.planner.btn_mission == False:
            self.planner.btn_mission = True
            self.thr_mission = threading.Thread(target=self.planner.func_mission, daemon= True)
            self.thr_mission.start()
        else:
            print(">>> [VirtualController] mission 수행 중..")
        

    def takeoff(self): #return: Tello의 receive 'OK' or 'FALSE'
         self.send_cmd('takeoff')



    #=====키보드를 입력했을 때 실행될 함수들=====
    def on_keypress_q(self, event):
        print("[VirtualController] Q")
        self.send_cmd("rc 0 0 0 0")
    
    
    def on_keypress_w(self, event):
        print("[VirtualController] W")
        val = self.tello_speed
        self.send_cmd(f"rc 0 0 {val} 0")



    def on_keypress_s(self, event):
        print("[VirtualController] S")
        val = -1*self.tello_speed
        self.send_cmd(f"rc 0 0 {val} 0")


    def on_keypress_a(self, event):
        print("[VirtualController] A")
        val = -1*self.tello_speed
        self.send_cmd(f"rc 0 0 0 {val}")


    def on_keypress_d(self, event):
        print("[VirtualController] D")
        val = self.tello_speed
        self.send_cmd(f"rc 0 0 0 {val}")


    def on_keypress_up(self, event):
        print("[VirtualController] UP")
        val = self.tello_speed
        self.send_cmd(f"rc 0 {val} 0 0")


    def on_keypress_down(self, event):
        print("[VirtualController] DOWN")
        val = -1*self.tello_speed
        self.send_cmd(f"rc 0 {val} 0 0")


    def on_keypress_left(self, event):
        print("[VirtualController] LEFT")
        val = -1*self.tello_speed
        self.send_cmd(f"rc {val} 0 0 0")


    def on_keypress_right(self, event):
        print("[VirtualController] RIGHT")
        val = self.tello_speed
        self.send_cmd(f"rc {val} 0 0 0")



    #=====스레드에서 실행될 함수=====


    #객체인식 화면을 출력하는 함수
    def func_print_video(self):
        while not self.stop_event.is_set():
            
            image = self.planner.get_info_11111Sensor_image()

            if self.panel_image is None: 
                self.panel_image = tkinter.Label(image=image)
                self.panel_image.image = image
                self.panel_image.pack(side="top", padx=10, pady=10)
            
            else:
                self.panel_image.configure(image=image)
                self.panel_image.image = image
        try:
            self.onClose()
        except:
            pass

    

    def func_but(self):
        start_cnt = 0
        while not self.stop_event.is_set():
            self.curr_value = GPIO.input(self.but_pin)
            
            if self.curr_value != self.prev_value:
                self.onClose()
                break
            
            #sleep(2)
            #start_cnt += 2
            #print(f"{start_cnt}..")
            #if start_cnt == 90:
            #    self.mission()
        
        try:
            self.onClose()
        except:
            pass



    #=====Tello에게 보낼 명령을 controller queue에 저장하는 함수=====
    def send_cmd(self, msg:str):
        self.socket8889.sendto(msg.encode('utf-8'), self.tello_address)
    

    #=====종료버튼을 클릭시 실행할 함수=====
    def onClose(self):
        self.socket8889.sendto("land".encode('utf-8'), self.tello_address)
        sleep(0.5)
        self.socket8889.sendto("motoroff".encode('utf-8'), self.tello_address)
        sleep(0.5)
        
        #종료 버튼 클릭
        print("종료중... >> stop event 실행")    
        self.stop_event.set()
        
        #화면 종료 
        self.root.quit() 
        
        #현 스레드 종료
        exit()

#완성
from COSMOS.Basemodel.ObjectDetector import ObjectDetector
import torch
import numpy as np
from PIL import Image,ImageTk
import cv2
from time import time
import os
import sys
sys.path.append(r'/home/cosmos/catkin_ws/src/donkey_ros/donkey_control/src/COSMOS/ObjectDetector/yolov5')



class YOLOv5(ObjectDetector):
    """
    객체인식을 담당하는 클래스(YOLOv5s 사용)
    """
    
    
    
    def __init__(self):

        #gpu initiate
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if self.device == 'cuda':
            torch.cuda.empty_cache()

        model_folder = r'/home/cosmos/catkin_ws/src/donkey_ros/donkey_control/src/COSMOS/ObjectDetector/yolov5'
        model_path = r'/home/cosmos/catkin_ws/src/donkey_ros/donkey_control/src/COSMOS/ObjectDetector/yolov5s.pt'
        cache_path = r'/home/cosmos/catkin_ws/src/donkey_ros/donkey_control/src/COSMOS/ObjectDetector/c_yolov5s.pt'

        #check
        if os.path.exists(cache_path):
            print("cache model")
            self.model = torch.load(cache_path, map_location=self.device)

        else:
            print("origin model")
            self.model = torch.hub.load(model_folder, 'custom', path=model_path, source='local')
            torch.save(self.model, cache_path)
            

        self.model.to(self.device)
        self.classes = self.model.names
         
        print(">>>>>>GPU 사용:",self.device)
        
    
    
    def detect_from_frame(self, frame): 
        """
        frame에서 객체를 감지하고, 윈도우를 적용한 image와 윈도우 리턴
        """
        #감지한 객체 윈도우들의 좌표를 저장할 리스트
        window_coor_list = []
        
        #프레임에서 감지한 객체들의 레이블들, 좌표들이 들어있는 리스트
        object_label_list, object_coor_list = self.score_frame(frame)
        
        #감지한 객체들을 frame에 표시
        n = len(object_label_list)
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = object_coor_list[i]
            if row[4] >= 0.2:
                #윈도우의 좌표 계산
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                          
                #frame에 그리기
                name = self.classes[int(object_label_list[i])]
                height = x2-x1
                width = y2-y1
                text = f"{name} h{height} w{width}"
                
                color_g = (0, 255, 0) #초록색
                color_r = (255,0,0) #빨간색
                color_b = (0,0,255) #파란색

                if name == 'keyboard':
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color_r, 2)
                    cv2.putText(frame, text, (x1, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_r, 2)
                elif name == 'person' or name == 'tv':
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color_b, 2)
                    cv2.putText(frame, text, (x1, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_b, 2)
                else:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color_g, 1)
                    cv2.putText(frame, text, (x1, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_g, 1)

                #윈도우의 좌표를 window_coor_list에 저장
                window_coor_list.append(((x1,y1), (x2,y2), name))   
                  
        
        #frame을 image로 변환
        image = Image.fromarray(frame)
        
        #image를 imagetk 형식으로 변환
        image = ImageTk.PhotoImage(image)
        
        return (image, window_coor_list)
    

    def score_frame(self, frame):
        # frame: 단일 프레임; numpy/list/tuple 형식
        # return: 프레임에서 모델이 감지한 객체의 레이블과 좌표
        self.model.to(self.device)
        frame = [frame]
        object_label_and_coor_list = self.model(frame)
        labels, coors = object_label_and_coor_list.xyxyn[0][:, -1].cpu().numpy(), object_label_and_coor_list.xyxyn[0][:, :-1].cpu().numpy()
        return labels, coors

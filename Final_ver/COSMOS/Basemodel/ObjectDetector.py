#완성
from abc import *
import numpy as np



class ObjectDetector(metaclass=ABCMeta):
    
    @abstractmethod
    def detect_from_frame(self, frame:np.fromstring): 
        """
        frame에서 객체를 감지하고, 윈도우를 적용한 image 및 윈도우의 (좌상단좌표, 우하단좌표)좌표를 리턴 
        """
        pass
#완성
from abc import *


class Actor(metaclass=ABCMeta):
    
    @abstractmethod
    def take_cmd_from_planner(self): 
        """
        Planner로부터 cmd를 가져온다
        """
        pass
    
    
    @abstractmethod
    def change_cmd_for_drone(self):
        """
        cmd를 Drone이 이해할 수 있는 cmd로 변경한다
        """
        pass
    
    @abstractmethod
    def send_to_actuator(self):
        """
        cmd를 Actuator에게 전송한다
        """
        pass
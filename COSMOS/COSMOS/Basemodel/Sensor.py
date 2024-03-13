#완성
from abc import *


class Sensor(metaclass=ABCMeta):
    
    @abstractmethod
    def take_data_from_sensor(self): 
        """
        센서로부터 data를 가져온다
        """
        pass
    
    @abstractmethod
    def change_data_to_info(self):
        """
        data를 Planner가 이해할 수 있는 info로 변경한다
        """
        pass
    
    @abstractmethod
    def save_to_planner(self):
        """
        info를 Planner에 저장한다
        """
        pass
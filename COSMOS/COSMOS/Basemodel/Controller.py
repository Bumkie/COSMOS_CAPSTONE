from abc import *


class Controller(metaclass=ABCMeta):
    
    """_summary_
    1) take_from_controller
    2) check_controller_value
    3) save_to_drone
    순으로 실행
    """
    
    @abstractmethod
    def __take_from_controller(self): 
        pass
    
    @abstractmethod
    def __check_controller_value(self):
        pass
    
    @abstractmethod
    def __save_to_drone(self):
        pass
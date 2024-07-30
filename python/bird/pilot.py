from tqdm import tqdm
import time

class PID_controller:
    def __init__(self, kp:float, ki:float, kd:float, cycle_time_seconds:float, i_limit:float = 10.0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.cycle_time_seconds = cycle_time_seconds
        self.i_limit = i_limit

        self.previous_error:float = 0.0
        self.previous_i:float = 0.0

    def calculate(self, actual:float, goal:float) -> float:
        """
        Performs a PID calculation
        :param actual: The reading (actual value)
        :param goal: Your goal for that value (setpoint)
        """

        error = goal - actual

        # P term
        P:float = error * self.kp

        # I term
        I:float = self.previous_i + (error * self.ki * self.cycle_time_seconds)
        I = max(min(I, self.i_limit), self.i_limit * -1)

        # D term
        D:float = self.kd * (error - self.previous_error) / self.cycle_time_seconds

        # set state variables for next time
        self.previous_error = error
        self.previous_i = I

        # return
        return P + I + D
    
    def reset(self) -> None:
        """Resets state variables."""
        self.previous_error = 0.0
        self.previous_i = 0.0

class Flap:
    def __init__(self, offset=0, max_angle=180, min_angle=-180) -> None:
        self.offset = offset
        self.max_angle = max_angle
        self.min_angle = min_angle

    def move_to(self, target_angle):
        target_angle += self.offset

        target_angle = min(target_angle, self.max_angle)
        target_angle = max(target_angle, self.min_angle)
        

        pass

    def check_up(self, test_range):
        print("Checking flap...")
        for i in range(test_range+1):
            self.move_to(i)
            time.sleep()

        for i in tqdm(range(-test_range, test_range+1)):
            self.move_to(i)
            time.sleep()

        print("Done !")

        


# Half automatic pilot for far remote controle 
class SoftPilot:
    def __init__(self):
        g = 9.81

        self.limit_ypr = [[float('-inf'), float('inf')], [-45, 45], [-60, 60]] #deg
        self.limit_acc = [[float('-inf'), float('inf')], [float('-inf'), float('inf')], [-4*g, 8*g]] #m.s-2 xyz based on corrected acc
        self.enable = False
    
    def enable(self):
        self.enable = True 

    def disable(self):
        self.enable = False

    # Rotation without alt change (angle deg)
    def rotate(self, angle:float):
        pass

    # Change pitch level
    def pitch(self, target: float):
        pass 
    
    # Important voir necessaire
    def takeoff(self, target_alt: float):
        pass

    # cv être chiant 
    def land(self):
        pass







def ypr_correction(target:list) -> list:

    ypr = []

    return ypr

        
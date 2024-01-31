class PID:
    def __init__(self, kp=0, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.last_error = 0
        self.integral = 0
        self.derivative = 0
    
    def output(self, target, current):
        error = target - current
        self.integral += error
        self.derivative = error - self.last_error
        self.last_error = error
        
        return self.kp * error + self.ki * self.integral + self.kd * self.derivative
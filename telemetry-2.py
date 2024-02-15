from dataclasses import dataclass
import serial

import numpy as np
import time
import matplotlib.pyplot as plt

@dataclass
class TestParams:
    P : float
    I : float 
    D : float
    SetPoint : float #degrees
    SampleRate : float = 1 #ms
    TestDuration : float = 1000 #ms

def parseval(string:str, index:int):
    return float(string[index].split(':')[1])

def run_test(ser:serial.Serial,params:TestParams):
    data = []
    test_complete = False
    ser.flush()

    if not ser.writable():
        raise AssertionError('Serial port is not writeable')
        
    
    ser.write(f'config_test,P,{params.P},I,{params.I},D,{params.D},set_points,{params.SetPoint},sample_rate,{params.SampleRate}'.encode())

    if ser.read_until('ack'.encode(),3).decode() != 'ack':
        raise AssertionError('Serial port did not respond with ack!')
    
    ser.write(f'start_test,duration,{params.TestDuration}'.encode())

    "time_ms:%lu,loop_dt:%lu,control_dt:%lu,read_dt:%lu,pid_success_flag:%i,position:%lf,pwm_duty_cycle:%lf,set_point:%f,velocity:%lf,current:%lf,torque_external:%lf\n"


    

    while test_complete == False:
        data_str = ser.readline().decode()
        print(data_str)
        if data_str == 'test_complete':
            test_complete = True
        else:
            data_arr = data_str.split(',')
            time_ms = parseval(data_arr, 0)
            loop_dt = parseval(data_arr, 1)
            control_dt = parseval(data_arr, 2)
            read_dt = parseval(data_arr, 3)
            pid_success_flag = parseval(data_arr, 4)
            position = parseval(data_arr, 5)
            pwm_duty_cycle = parseval(data_arr, 6)
            set_point = parseval(data_arr, 7)
            velocity = parseval(data_arr, 8)
            current = parseval(data_arr, 9)
            torque_external = parseval(data_arr, 10)

            if len(data_arr) != 2:
                AssertionError("Telemetry data processe with length")



if __name__ == "__main__":
    pos = []
    cnt = 0
        # print("hello")
    with serial.Serial('/dev/tty.usbserial-1440', 115200, timeout=1) as ser:
        while True:
            read_str = ser.readline()
            print(read_str)
            try:
                data_str = read_str.decode()
            except Exception as e:
                print(e)
                continue
            # print(data_str)
            data_arr = data_str.split(',')
            if data_str == 'test_complete':
                test_complete = True
            elif len(data_arr) == 11:
                time_ms = parseval(data_arr, 0)
                loop_dt = parseval(data_arr, 1)
                control_dt = parseval(data_arr, 2)
                read_dt = parseval(data_arr, 3)
                pid_success_flag = parseval(data_arr, 4)
                position = parseval(data_arr, 5)
                pwm_duty_cycle = parseval(data_arr, 6)
                set_point = parseval(data_arr, 7)
                velocity = parseval(data_arr, 8)
                current = parseval(data_arr, 9)
                torque_external = parseval(data_arr, 10)
                print(position,'\n')

    # plt.plot(pos)
    # plt.show()
    
    
            
            
            

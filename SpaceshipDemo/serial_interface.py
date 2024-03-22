# game_interface.py v0.3
#   A python serial based protocol with an esp32 for a haptic virtual environment
# Author: Yousif El-Wishahy (ywishahy@student.ubc.ca)

#import async for and serial_asycnc for protocol
import asyncio
import serial_asyncio

from dataclasses import dataclass
import dataclasses

import queue


@dataclass
class TelemetryFrame:
    """
    Telemetry frame received from the esp32

    Telemetry frame formatting: "/*<ID>,%lu,%lu,%lu,%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf*/\n", 
    t.timestamp_ms, t.loop_dt, t.control_dt, t.read_dt, 
    t.pwm_duty_cycle, t.pwm_frequency, 
    t.position, t.velocity, t.filtered_velocity, t.current, t.filtered_current, t.torque_external, t.torque_control, t.torque_net,
    t.setpoint.pos, t.setpoint.vel, t.setpoint.accel, t.setpoint.torque,
    t.Kp, t.Ki, t.Kd, t.impedance.K, t.impedance.B, t.impedance.J
    """
    timestamp_ms:int=0
    loop_dt:int=0
    control_dt:int=0
    read_dt:int=0

    #motor signal state
    pwm_duty_cycle:float=0
    pwm_frequency:float=0

    #state based on sensors
    position:float=0
    velocity:float=0
    filtered_velocity:float=0
    current:float=0
    filtered_current:float=0
    torque_external:float=0
    torque_control:float=0
    torque_net:float=0

    #setpoints
    position_setpoint:float=0
    velocity_setpoint:float=0
    accel_setpoint:float=0
    torque_setpoint:float=0
    
    #control params
    Kp:float=0
    Ki:float=0
    Kd:float=0

    #impedance params
    impedance_K:float=0
    impedance_B:float=0
    impedance_J:float=0

#serial async protocol for interfacing with the twiddlerino
class TwidSerialInterfaceProtocol(asyncio.Protocol):
    """
    Vritual Enviornment Serial Protocol based on asyncio for async serial.
    Features:
        Processes bytes into TelemetryFrame classes for use in the virtual environment.
        Functions avaialable to update config /control targers  on esp32 such as setpoint, and gains.
    """
    frame_count:int = 0
    err_frame_count:int = 0
    last_frame:TelemetryFrame = None
    frames:queue.Queue = queue.Queue(1000)
    buffer:bytes = b''
    transport:serial_asyncio.SerialTransport
    datafields:list[str] = [field.name for field in dataclasses.fields(TelemetryFrame)]

    def connection_made(self, transport) -> None:
        self.frames = queue.Queue(1000)
        self.frame_count = 0
        self.err_frame_count = 0
        self.buffer = b''
        self.transport = transport
        self.transport.write(b'telemetry_disable\n')
        self.transport.resume_reading()
        print('port opened', transport)

    def data_received(self, data) -> None:
        #add buffer (it could be empty)
        data = self.buffer + data
        self.buffer = b''

        #check if we are at start of frame
        if data[0:2] == b'/*':
            #check if we are at end of frame
            if b'*/' in data:
                #extract complete frame
                spl = data.split(b'*/')
                if(self.frames.full()):
                    try:
                        self.frames.get_nowait()
                    except:
                        pass
                try:
                    frame = self.bytes2frame(spl[0])
                    self.frames.put(frame)
                    self.last_frame = frame
                    self.frame_count+=1
                except Exception as errmsg:
                    print(errmsg)
                    self.err_frame_count+=1
                #add any remaining bytes to buffer
                self.buffer = b',' + b','.join(spl[1:-1])
            else:
                #add to buffer since incomplete
                self.buffer = data

    def pause_reading(self) -> None:
        # This will stop the callbacks to data_received
        self.transport.pause_reading()

    def resume_reading(self) -> None:
        # This will start the callbacks to data_received again with all data that has been received in the meantime.
        self.transport.resume_reading()
    
    def flush_write_buffer(self) -> None:
        self.transport.flush()
        
    def write(self, data:bytes) -> None:
        self.transport.write(data)
    
    def update_setpoint(self, position:float=0, velocity:float=0, accel:float=0, torque:float=0) -> None:
        self.transport.write(bytes(f'set_setpoint,{position},{velocity},{accel},{torque},\n',"utf-8"))

    def update_impedance(self, K :float=0, B :float=0, J :float= 0) -> None:
        self.transport.write(bytes(f'set_impedance,{K},{B},{J},\n',"utf-8"))

    def update_pid(self, Kp :float=0, Ki :float=0, Kd :float= 0) -> None:
        self.transport.write(bytes(f'set_pid,{Kp},{Ki},{Kd},\n',"utf-8"))
        
    def configure_controller_default(self) -> None:
        self.transport.write(bytes(f'reset\n',"utf-8"))
        self.transport.write(b'telemetry_disable\n')
        self.transport.write(bytes(f'set_mode,position,\n',"utf-8"))
        self.transport.write(bytes(f'set_pid,5.0,0.0,0.0,\n',"utf-8"))
        
    def formattedbytes2frame(self, data:bytes) -> TelemetryFrame :
        spl = data.decode("utf-8").split(',')
        # print(spl, len(spl))
        if len(spl) != 25:
            raise Exception(f'cannot convert bytes to telemetry frame! len: {len(spl)}')
        t = TelemetryFrame()
        for item in spl[1:-1]:
            spl2 = item.split(':')
            name = spl2[0]
            val = float(spl2[1])
            setattr(t,name,val)
        return t

    def bytes2frame(self, data:bytes) -> TelemetryFrame :
        spl = data.decode("utf-8").split(',')
        # print(spl, len(spl))
        if len(spl) != 19:
            raise Exception(f'cannot convert bytes to telemetry frame! len: {len(spl)}')
        t = TelemetryFrame()
        count: int = 0
        for item in spl[1:-1]:
            setattr(t,self.datafields[count],float(item))
            count+=1
        return t
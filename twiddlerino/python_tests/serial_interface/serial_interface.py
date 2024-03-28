# game_interface.py v0.3
#   A python serial based protocol with an esp32 for a haptic virtual environment
#   Also shares telemetry data over tcp socket - this can be accessed by other GUI software such as serial studio
# Author: Yousif El-Wishahy (ywishahy@student.ubc.ca)

#import async for and serial_asycnc for protocol
import asyncio
import serial_asyncio

from dataclasses import dataclass
import dataclasses

import queue

import socket

SERIAL_STUDIO_HOST = '127.0.0.1'
SERIAL_STUDIO_PORT = 15555

@dataclass
class TelemetryFrame:
    """
    Telemetry frame received from the esp32

    Telemetry frame formatting: "/*<ID>,%lu,%lu,%lu,%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lu,%lu*/\n", 
    t.timestamp_ms, t.loop_dt, t.control_dt, t.read_dt, 
    t.pwm_duty_cycle, t.pwm_frequency, 
    t.position, t.velocity, t.filtered_velocity, t.current, t.filtered_current, t.torque_external, t.torque_control, t.torque_net,
    t.setpoint.pos, t.setpoint.vel, t.setpoint.accel, t.setpoint.torque,
    t.Kp, t.Ki, t.Kd, t.impedance.K, t.impedance.B, t.impedance.J, t.nframes_sent_queue, nframes_sent_serial
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
    
    #telemetry counts
    nframes_sent_queue:int=0
    nframes_sent_serial:int=0

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
    socket_buffer:queue.Queue = queue.Queue(1000)
    _instance = None

    def __init__(self):
        super(TwidSerialInterfaceProtocol, self).__init__()
        TwidSerialInterfaceProtocol._instance = self

    def connection_made(self, transport) -> None:
        self.frames = queue.Queue(1000)
        self.frame_count = 0
        self.err_frame_count = 0
        self.buffer = b''
        self.socket_buffer:queue.Queue = queue.Queue(1000)
        self.transport = transport
        self.transport.resume_reading()
        print('port opened', transport)

    def data_received(self, data) -> None:
        if self.socket_buffer.full():
            try:
                self.socket_buffer.get_nowait()
            except Exception as errmsg:
                print(errmsg)
        self.socket_buffer.put(data)

        #add buffer (it could be empty)
        data = self.buffer + data
        self.buffer = b''

        #check if we are at start of frame
        if b'/*' in data:
            if b'*/' in data:
                #extract complete frame
                spl = data.split(b'/*')
                for seg in spl:
                    #check if we are at end of frame
                    if seg == b'':
                        continue
                    elif b'*/' not in seg:
                        if seg == spl[0]:
                            self.buffer += seg
                        else:
                            self.buffer += b'/*' + seg
                    else:
                        try:
                            frame = self.bytes2frame(seg.split(b'*/')[0])
                            self.frame_count+=1
                            if self.frames.full():
                                self.frames.get_nowait()
                            self.frames.put(frame)
                            self.last_frame = frame
                        except Exception as errmsg:
                            print(errmsg)
                            self.err_frame_count+=1
            else:
                #add to buffer since incomplete
                self.buffer += data

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
    
    def bytes2frame(self, data:bytes) -> TelemetryFrame :
        spl = data.decode("utf-8").split(',')
        # print(spl, len(spl))
        if len(spl) != 27:
            raise Exception(f'cannot convert bytes to telemetry frame! len: {len(spl)}')
        t = TelemetryFrame()
        count: int = 0
        for item in spl[1:]:
            setattr(t,self.datafields[count],float(item))
            count+=1
        return t

    def control_stop(self):
        self.transport.write(bytes(f'stop\n',"utf-8"))
    
    def control_reset(self):
        self.transport.write(bytes(f'reset\n',"utf-8"))

    def esp32_reboot(self):
        self.transport.write(bytes(f'reboot\n',"utf-8"))

    def motor_stop(self):
        self.transport.write(bytes(f'set_dutycycle,0,\n',"utf-8"))
    
    def update_setpoint(self, position:float=0, velocity:float=0, accel:float=0, torque:float=0) -> None:
        self.transport.write(bytes(f'set_setpoint,{position},{velocity},{accel},{torque},\n',"utf-8"))

    def update_impedance(self, K :float=0, B :float=0, J :float= 0) -> None:
        self.transport.write(bytes(f'set_impedance,{K},{B},{J},\n',"utf-8"))

    def update_pid(self, Kp :float=0, Ki :float=0, Kd :float= 0) -> None:
        self.transport.write(bytes(f'set_pid,{Kp},{Ki},{Kd},\n',"utf-8"))
        
    def configure_controller_default(self) -> None:
        self.transport.write(bytes(f'reset\n',"utf-8"))
        self.transport.write(b'telemetry_enable\n')
        self.transport.write(bytes(f'set_mode,position,\n',"utf-8"))
        self.transport.write(bytes(f'set_pid,1.0,0.0,0.0,\n',"utf-8"))

    def configure_controller_speed(self) -> None:
        self.transport.write(bytes(f'reset\n',"utf-8"))
        self.transport.write(b'telemetry_enable\n')
        self.transport.write(bytes(f'set_mode,velocity,\n',"utf-8"))
        self.transport.write(bytes(f'set_pid,1.0,0.0,0.0,\n',"utf-8"))

    def turn_off_control(self) -> None:
        self.transport.write(bytes(f'reset\n',"utf-8"))
        self.transport.write(b'telemetry_enable\n')
        self.transport.write(bytes(f'set_mode,no_control,\n',"utf-8"))

    def update_pwm(self, duty_cycle:float = 0) -> None:
        self.transport.write(bytes(f'set_dutycycle,{duty_cycle},\n',"utf-8"))
    
    def update_telem_sample_rate(self, rate:int=20) -> None:
        self.transport.write(bytes(f'set_telemsamplerate,{rate},\n',"utf-8"))

async def run_socket_server_async(delay=0.01):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((SERIAL_STUDIO_HOST, SERIAL_STUDIO_PORT))
    server.listen(1)
    server.setblocking(False)
    loop = asyncio.get_event_loop()

    client, _ = await loop.sock_accept(server)
    print(f'Socket connection established on {client.getsockname()}')
    twid = None

    while True:
        if twid is None:
            if TwidSerialInterfaceProtocol._instance is None:
                raise(f'Cannot grab instance of {TwidSerialInterfaceProtocol.__name__}')
            twid = TwidSerialInterfaceProtocol._instance

        if not twid.socket_buffer.empty():
            data = twid.socket_buffer.get()
            try:
                await loop.sock_sendall(client, data)
            except Exception as errmsg:
                print(errmsg)
                client.close()
                print('Waiting to accept new socket connection.')
                client, _ = await loop.sock_accept(server)
                print(f'Socket connection re-established on {client.getsockname()}')

        await asyncio.sleep(delay)

def run_socket_server_thread(test_instance):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((SERIAL_STUDIO_HOST, SERIAL_STUDIO_PORT))
    server.listen()
    twid:TwidSerialInterfaceProtocol = None

    while twid is None:
        try:
            twid:TwidSerialInterfaceProtocol = test_instance.twid._instance
        except Exception as errmsg:
            pass
    print(f'Grabbed instance of {TwidSerialInterfaceProtocol.__name__}')
        
    print('Waiting to accept new socket connection.')
    client, _ = server.accept()
    print(f'Socket connection re-established on {client.getsockname()}')

    while not twid.transport.is_closing():
        data = twid.socket_buffer.get()
        try:
            client.sendall(data)
        except Exception as errmsg:
            print(errmsg)
            client.close()
            print('Waiting to accept new socket connection.')
            client, _ = server.accept()
            print(f'Socket connection re-established on {client.getsockname()}')
    print('Serial port closed.. closing serial studio socket now.')
    client.close()
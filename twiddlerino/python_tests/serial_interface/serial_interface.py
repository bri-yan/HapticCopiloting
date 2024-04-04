# serial_interface.py v2.3
#   A python serial based protocol with an esp32 for a haptic virtual environment
#   Also shares telemetry data over tcp socket - this can be accessed by other GUI software such as serial studio
#
#   Author: Yousif El-Wishahy (ywishahy@student.ubc.ca)
#   2024-04-03
import logging

#import async for and serial_asycnc for protocol
import asyncio
import serial_asyncio

#data storage
from dataclasses import dataclass
from enum import Enum
import dataclasses

#thread safe data transfer
import queue

#to open a socket and echo bytes to external GUI application
import socket

#decoding
import re

import time

import os

import pandas

#external gui socket address
SERIAL_STUDIO_HOST = '127.0.0.1'
SERIAL_STUDIO_PORT = 15555

TELEMETRY_FRAME_LENGTH = 29

_LOG_NAME = 'TWIDDLERINO_SERIAL'
_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..','data','logs')
_LOG_FILE = os.path.join(_LOG_DIR, f'{_LOG_NAME}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.log')
_TELEM_LOG_FILE = os.path.join(_LOG_DIR, f'telemetry_frame_{time.strftime('%Y-%m-%d_%H-%M-%S')}.log')
_SERIALDEBUG_LOG_FILE = os.path.join(_LOG_DIR, f'serial_debug_dump_{time.strftime('%Y-%m-%d_%H-%M-%S')}.log')

#python copy of control_type_t in esp32 firmware
class ControlType(Enum):
    POSITION_CTRL = 0, 'position',                                  #position pid control
    VELOCITY_CTRL = 1, 'velocity',                                  #velocity pid control
    TORQUE_CTRL = 2, 'torque',                                      #torque pid control
    IMPEDANCE_CTRL = 3, 'impedance',                                #cascaded impedance control with torque pid control inner loop
    ADMITTANCE_CTRL = 4, 'admittance',                              #cascaded admittance control with position pid control inner loop
    NO_CTRL = 5, 'no_control',                                      #no control, telemetry only
    IMPEDANCE_CTRL_SPRING = 6, 'impedance_spring',                  #impedance control simplified to spring mode and accounting for friction torque
    IMPEDANCE_CTRL_DAMPING = 7, 'impedance_damping',                #impedance control simplified to damer mode and accounting for friction torque
    IMPEDANCE_CTRL_SPRING_DAMPING = 8, 'impedance_spring_damping',  #impedance control with spring and damping and accounting for friction torque
    IMPEDANCE_CTRL_IGNORE_T_EXT = 9, 'impedance_ignore_t_ext',      #impedance control with spring, damper, intertia and accounting for friction torque
    
#python copy of cmd_type_t in esp32 firmware
class CommandType(Enum):
    NA_CMD = -1
    STOP = 1
    RESET= 2
    REBOOT= 3
    TELEM_ENABLE= 4
    TELEM_DISABLE= 5
    SET_SETPOINT= 6
    SET_DUTYCYCLE= 7
    SET_PID= 8
    SET_IMPEDANCE= 9
    SET_MODE= 10
    SET_TELEMSAMPLERATE= 11
    SET_MULTISETPOINT_POSITION = 12
    SET_MULTISETPOINT_VELOCITY = 13
    SET_MULTISETPOINT_ACCELERATION = 14
    SET_MULTISETPOINT_TORQUE = 15

@dataclass
class TelemetryFrame:
    """
    Telemetry frame received from the esp32

    Telemetry frame formatting: "/*TWIDDLERINO_TELEMETRY,%lu,%lu,%lu,%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lu,%lu,%i*/\n", 
            t.timestamp_ms, t.loop_dt, t.control_dt, t.read_dt, 
            t.pwm_duty_cycle, t.pwm_frequency, 
            t.position, t.velocity, t.filtered_velocity, t.current, t.filtered_current, t.torque_external, t.torque_control, t.torque_net,
            t.setpoint.pos, t.setpoint.vel, t.setpoint.accel, t.setpoint.torque,
            t.Kp, t.Ki, t.Kd, t.impedance.K, t.impedance.B, t.impedance.J, t.nframes_sent_queue, nframes_sent_serial, (int16_t)t.control_type
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
    
    #control type id
    control_type_val:int=0
    control_type:ControlType= ControlType(ControlType.NO_CTRL)
    
    #adc volts
    current_sens_adc_voltage:float = 0

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
    frames:queue.Queue = queue.Queue(0)
    buffer:bytes = b''
    transport:serial_asyncio.SerialTransport
    datafields:list[str] = [field.name for field in dataclasses.fields(TelemetryFrame)]
    control_value_fields:list[tuple[int,str]] = [e.value for e in ControlType]
    socket_write_buffer:queue.Queue = queue.Queue(100)
    _instance = None
    
    #re
    cmd_pattern = rb'/\*TWIDDLERINO_ACK,(-?\d+)\*/'
    telem_pattern = rb'/\*\*TWIDDLERINO_TELEMETRY,([^/]+?)\*\*/'

    def __init__(self):
        super(TwidSerialInterfaceProtocol, self).__init__()
        TwidSerialInterfaceProtocol._instance = self
        self._stop_flag = asyncio.Event()
        self._conn_flag = asyncio.Event()
        self._ack_flag = asyncio.Event()
        self._telem_after_reboot = asyncio.Event()
        self._reboot_flag = asyncio.Event()
        self._wait_for_param_flag = asyncio.Event()
        self._wait_for_name = None
        self._wait_for_value = None

        #handle logging creation
        fh = logging.FileHandler(_LOG_FILE)
        fh.setFormatter(logging.Formatter('%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s','%H:%M:%S'))
        fh.setLevel(logging.DEBUG)
        sh = logging.StreamHandler()
        sh.setLevel(logging.ERROR)

        self._logger = logging.getLogger(_LOG_NAME)
        self._logger.setLevel(logging.DEBUG)
        self._logger.addHandler(fh)
        self._logger.addHandler(sh)

        tfh = logging.FileHandler(_TELEM_LOG_FILE)
        tfh.setFormatter(logging.Formatter('%(message)s','%H:%M:%S'))
        self._telem_logger = logging.getLogger(f'{_LOG_NAME}_TELEMETRY')
        self._telem_logger.setLevel(logging.DEBUG)
        self._telem_logger.addHandler(tfh)

        tfh = logging.FileHandler(_SERIALDEBUG_LOG_FILE)
        tfh.setFormatter(logging.Formatter('%(asctime)s,%(msecs)d %(message)s','%H:%M:%S'))
        self._serial_dump_logger = logging.getLogger(f'{_LOG_NAME}_DUMP')
        self._serial_dump_logger.setLevel(logging.DEBUG)
        self._serial_dump_logger.addHandler(tfh)
        
        self._logger.info(f'{TwidSerialInterfaceProtocol.__name__} __init__ called')

    def __del__(self):
        self._logger.info(f'{TwidSerialInterfaceProtocol.__name__} __del__ called')

    def connection_made(self, transport) -> None:
        self._loop = asyncio.get_event_loop()
        self.frames = queue.Queue(0)
        self.frame_count = 0
        self.err_frame_count = 0
        self.buffer = b''
        self.socket_write_buffer:queue.Queue = queue.Queue(1000)
        self.transport = transport
        self.transport.resume_reading()
        self._conn_flag.set()
        self._logger.info(f'port opened {transport}')
    
    def connection_lost(self, exc):
        self._logger.info(f'{TwidSerialInterfaceProtocol.__name__} connection lost')
        self._stop_flag.set()
        self._logger.info(f'set stop flag @ {self._stop_flag}')

    def data_received(self, data) -> None:
        #echo to socket buffer
        if self.socket_write_buffer.full():
            try:
                self.socket_write_buffer.get_nowait()
            except Exception as errmsg:
                print(errmsg)
        self.socket_write_buffer.put(data)

        #add buffer (it could be empty)
        data = self.buffer + data
        self.buffer = b''
        
        #split by lines and try to decode each line
        data_lines = data.split(b'\n')
        for i in range(len(data_lines)):
            processed = False
            line = data_lines[i]

            #check if this is a command ack string
            if b'/*T' in line:
                match = re.match(self.cmd_pattern, line)
                if match:
                    ack_val = int(match.group(1))
                    self._last_ack_payload = CommandType(ack_val)
                    self._ack_flag.set()
                    processed = True
                    self._logger.debug(f'set ack flag @ {self._ack_flag}')
                    self._logger.debug(f'ack payload received {self._last_ack_payload}')
            #check if this is a telemetry string
            elif b'/**' in line or b'**/' in line:
                match = re.match(self.telem_pattern, line)
                if match:
                    stripped_telem_bytes = match.group(1)
                    frame = None

                    try:
                        #try to decode frame 
                        frame = self.bytes2frame(stripped_telem_bytes)
                    except Exception as errmsg:
                        self.err_frame_count+=1
                    
                    #if we decoded a frame
                    if frame is not None:
                        self._telem_logger.info(frame)
                        self.last_frame = frame
                        self.frame_count+=1

                        #add to queue
                        if self.frames.full():
                            self._logger.warn(f'telemetry frame queue full (qsize {self.frames.qsize()})')
                            self.frames.get_nowait()
                        self.frames.put(frame)

                        #handle telemetry received after reboot flag
                        if self._reboot_flag.is_set():
                            self._reboot_flag.clear()
                            self._telem_after_reboot.set()
                            self._logger.debug(f'cleared reboot flag @ {self._reboot_flag}')
                            self._logger.debug(f'set telem_after_reboot flag @ {self._telem_after_reboot}')

                        #handle waiting for specific parameter flag
                        if self._wait_for_name is not None:
                            if getattr(frame, self._wait_for_name) == self._wait_for_value:
                                self._wait_for_param_flag.set()
                                self._logger.debug(f'wait for param {self._wait_for_name} has matched value {self._wait_for_value}')
                                self._logger.debug(f'set wait_for_param_flag flag @ {self._wait_for_param_flag}')
                        
                        processed = True
            if not processed and len(line) > 1:
                self._serial_dump_logger.debug(line)
                if i < len(data_lines) - 1:
                    self.buffer += line + b'\n'
                else:
                    self.buffer += b'\n' + line

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
        if len(spl) != TELEMETRY_FRAME_LENGTH - 1:
            raise Exception(f'cannot convert bytes to telemetry frame! len: {len(spl)}')
        t = TelemetryFrame()
        count: int = 0
        for item in spl:
            setattr(t,self.datafields[count],float(item))
            count+=1
        for num, name in TwidSerialInterfaceProtocol.control_value_fields:
            if num == t.control_type_val:
                t.control_type = ControlType((num,name))
        return t

    #blocking function that returns true when command to twid is acknowledged with correct code
    #this is a awaitable function
    #if blocking is set to false, send command will not wait for acknowledgement
    async def send_cmd(self, cmd_bytes:bytes, cmd_type:CommandType, blocking=True, timeout=0.25, reboot_timeout=5) -> bool:
        self._last_ack_payload = CommandType.NA_CMD
        self._ack_flag.clear()
        self.write(cmd_bytes)
        self._logger.info(f'sent command of type {cmd_type}\tcommand frame bytes:\t{cmd_bytes}')

        if blocking:
            try:
                await asyncio.wait_for(self._ack_flag.wait(), timeout=timeout)
            except Exception as errmsg:
                print(errmsg)
                self._logger.warn(f'wait for command timed out for command of type {cmd_type}')
                return False
            
            if cmd_type == CommandType.RESET or cmd_type == CommandType.REBOOT:
                self._telem_after_reboot.clear()
                self._logger.debug(f'cleared telem_after_reboot flag @ {self._telem_after_reboot}')
                self._reboot_flag.set()
                self._logger.debug(f'set reboot flag @ {self._reboot_flag}')

                try:
                    await asyncio.wait_for(self._telem_after_reboot.wait(), timeout=reboot_timeout)
                    self._telem_after_reboot.clear()
                    self._reboot_flag.clear()
                except Exception as errmsg:
                    print(errmsg)
                    return False

            if self._last_ack_payload == cmd_type:
                return True

        return False
    
    #wait for telemetry parameter to equal a certain value
    #this is a awaitable function
    async def wait_for_param(self, name:str, value, timeout=0.5):
        self._wait_for_name = name
        self._wait_for_value = value
        self._wait_for_param_flag.clear()
        success = False
        try:
            success = await asyncio.wait_for(self._wait_for_param_flag.wait(), timeout=timeout)
        except Exception as errmsg:
            print(errmsg)
        self._wait_for_param_flag.clear()
        self._wait_for_name = None
        self._wait_for_value = None
        return success
    
    #collect telemetry for defined duration +/- 1ms and return a list of collected telemetry
    #this is a awaitable function
    #set collect_old_buffer to true if you want to get all the frames currently in buffer
    # otherwise buffer is cleared
    async def collect_telem(self, duration=1, collect_old_buffer=False) -> pandas.DataFrame:
        out :list[TelemetryFrame] = []
        with self.frames.mutex:
            if collect_old_buffer:
                out = list(self.frames.queue)
                self._logger.debug(f'copied over {len(out)} items from frame queue to output list of collect_telem')
            self.frames.queue.clear()
            self._logger.warn(f'cleared frame queue')
        start = time.time()
        start_size = self.frames.qsize()
        
        while time.time() < start + duration:
            if self.frames.qsize() > start_size:
                out.append(self.frames.get())
            await asyncio.sleep(1e-3)
        
        self._logger.debug(f'collected {len(out)} telemetry frames during {duration}s interval')
        
        out_dict = {}
        for t in out:
            for attr_name in self.datafields:
                if attr_name in out_dict.keys():
                    out_dict[attr_name].append(getattr(t, attr_name))
                else:
                    out_dict[attr_name] = []
        out_dataframe = pandas.DataFrame(out_dict)
    
        return out_dataframe
    
    def clear_frames(self):
        cnt = self.frames.qsize()
        with self.frames.mutex:
            self.frames.queue.clear()
        self._logger.warn(f'cleared {cnt} items from frame queue')

    async def end(self):
        await self.motor_stop()
        await self.esp32_reboot()
        self._stop_flag.set()
        self.transport.close()
        self._logger.info(f'end called')
        self.__del__()

    async def control_stop(self):
        return await self.send_cmd(bytes(f'stop\n',"utf-8"), CommandType.STOP)
    
    async def control_reset(self):
        return await self.send_cmd(bytes(f'reset\n',"utf-8"), CommandType.RESET)

    async def esp32_reboot(self):
        return await self.send_cmd(bytes(f'reboot\n',"utf-8"), CommandType.REBOOT)

    async def motor_stop(self):
        return await self.send_cmd(bytes(f'set_dutycycle,0,\n',"utf-8"), CommandType.SET_DUTYCYCLE)
        
    async def update_control_type(self, type:ControlType):
        return await self.send_cmd(bytes(f'set_mode,{type.value[1]},\n',"utf-8"), CommandType.SET_MODE)
    
    async def update_setpoint(self, position:float=0, velocity:float=0, accel:float=0, torque:float=0):
        return await self.send_cmd(bytes(f'set_setpoint,{position},{velocity},{accel},{torque},\n',"utf-8"), CommandType.SET_SETPOINT)

    async def update_pos_setpoint_mulitple(self, sp:list[float]):
        return await self.send_cmd(bytes(f'set_multisetpoint,position,{len(sp)},{','.join(map(str, sp))},\n',"utf-8"), CommandType.SET_MULTISETPOINT_POSITION, timeout=100)

    async def update_vel_setpoint_mulitple(self, sp:list[float]):
        return await self.send_cmd(bytes(f'set_multisetpoint,velocity,{len(sp)},{','.join(map(str, sp))},\n',"utf-8"), CommandType.SET_MULTISETPOINT_VELOCITY)

    async def update_accel_setpoint_mulitple(self, sp:list[float]):
        return await self.send_cmd(bytes(f'set_multisetpoint,acceleration,{len(sp)},{','.join(map(str, sp))},\n',"utf-8"), CommandType.SET_MULTISETPOINT_ACCELERATION)

    async def update_torque_setpoint_mulitple(self, sp:list[float]):
        return await self.send_cmd(bytes(f'set_multisetpoint,torque,{len(sp)},{','.join(map(str, sp))},\n',"utf-8"), CommandType.SET_MULTISETPOINT_TORQUE)

    async def update_impedance(self, K :float=0, B :float=0, J :float= 0):
        return await self.send_cmd(bytes(f'set_impedance,{K},{B},{J},\n',"utf-8"), CommandType.SET_IMPEDANCE)

    async def update_pid(self, Kp :float=0, Ki :float=0, Kd :float= 0):
        return await self.send_cmd(bytes(f'set_pid,{Kp},{Ki},{Kd},\n',"utf-8"), CommandType.SET_PID)

    async def update_dutycycle(self, duty_cycle:float = 0):
        return await self.send_cmd(bytes(f'set_dutycycle,{duty_cycle},\n',"utf-8"), CommandType.SET_DUTYCYCLE)
    
    async def update_telem_sample_rate(self, rate:int=20):
        return await self.send_cmd(bytes(f'set_telemsamplerate,{rate},\n',"utf-8"), CommandType.SET_TELEMSAMPLERATE)

async def run_socket_server_async(twid:TwidSerialInterfaceProtocol, delay=0):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((SERIAL_STUDIO_HOST, SERIAL_STUDIO_PORT))
    server.listen(1)
    server.setblocking(False)
    loop = twid._loop
    client = None
    twid._logger.debug('[socket_server] Waiting to accept new socket connection.')
    while not twid._stop_flag.is_set():
        if client is None:
            try:
                client, _ = await asyncio.wait_for(loop.sock_accept(server), 0.01)
                twid._logger.debug(f'[socket_server] Socket connection re-established on {client.getsockname()}')
            except:
                pass
        else:
            if not twid.socket_write_buffer.empty():
                data = twid.socket_write_buffer.get()
                try:
                    await loop.sock_sendall(client, data)
                except Exception as errmsg:
                    print(errmsg)
                    client.close()
                    client = None

            try:
                recv_bytes = bytes(1024)
                await asyncio.wait_for(loop.sock_recv_into(client, recv_bytes), timeout=0.01)
                if len(recv_bytes) > 0:
                    twid.write(recv_bytes)
            except Exception as errmsg:
                pass
        await asyncio.sleep(delay)

    if client is not None:
        client.close()
    server.close()
    twid._logger.debug('[socket_server] Socket closed.')
    
async def start_protocol(loop, serial_port, baud_rate) -> TwidSerialInterfaceProtocol:
    _, twid = await serial_asyncio.create_serial_connection(loop, TwidSerialInterfaceProtocol, serial_port, baudrate=baud_rate)
    await twid._conn_flag.wait()
    return twid

def run_test(serial_port, baud_rate, *args):
    loop = asyncio.get_event_loop()
    twid = loop.run_until_complete(start_protocol(loop,serial_port,baud_rate))
    twid._logger.info(f'running functions test(s):\t {[arg.__name__ for arg in args]}')
    loop.run_until_complete(asyncio.gather(run_socket_server_async(twid), *[arg(twid) for arg in args]))
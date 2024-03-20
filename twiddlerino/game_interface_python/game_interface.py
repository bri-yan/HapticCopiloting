import asyncio
import serial_asyncio
import queue
from dataclasses import dataclass

@dataclass
class TelemetryFrame:
    timestamp_ms:int
    position:float
    velocity:float
    position_setpoint:float
    velocity_setpoint:float

#serial async protocol for interfacing with the twiddlerino
class GameInterfaceProtocol(asyncio.Protocol):
    frame_count:int = 0
    last_frame:TelemetryFrame = None
    frames:queue.Queue = queue.Queue(1000)
    buffer:bytes = b''
    transport:serial_asyncio.SerialTransport

    def connection_made(self, transport) -> None:
        self.frames = queue.Queue(1000)
        self.frame_count = 0
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
                #add any remaining bytes to buffer
                self.buffer = b''.join(spl[1:-1])
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
        
    def bytes2frame(self, data:bytes) -> TelemetryFrame :
        spl = data.decode("utf-8").split(',')
        # print(spl, len(spl))
        if len(spl) != 7:
            raise Exception("cannot convert bytes to telemetry frame!")
        t = TelemetryFrame(0,0,0,0,0)
        for item in spl[1:-1]:
            spl2 = item.split(':')
            name = spl2[0]
            val = float(spl2[1])
            setattr(t,name,val)
        return t

#example usage
async def reader():
    game_interface: GameInterfaceProtocol
    transport, game_interface = await serial_asyncio.create_serial_connection(loop, GameInterfaceProtocol, 'COM9', baudrate=500000)
    await asyncio.sleep(0.25) #wait for connection to init
    
    #update parameters over serial;
    game_interface.update_setpoint(position=1.0, velocity=1e-5, torque= 0.0)
    game_interface.update_impedance(K=1e-3, B=1e-2, J=1e-6)
    game_interface.update_pid(1.0 ,0.0, 0.0)
    
    #loop and read latest frames
    while True:
        #check latest variable
        print(f'frame #:{game_interface.frame_count}, frame:{game_interface.last_frame}')
        
        #or alternatively, pop the queue, could throw exception
        try:
            print(f'frame #:{game_interface.frames.qsize()}, frame:{game_interface.frames.get_nowait()}')
        except:
            pass
        await asyncio.sleep(0.01)
            
loop = asyncio.get_event_loop()
loop.run_until_complete(reader())
loop.close()

import asyncio
import serial_asyncio
from game_interface import GameInterfaceProtocol

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

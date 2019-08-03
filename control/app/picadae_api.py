import os,sys,argparse,yaml,types,select,serial,logging,time,datetime
from enum import Enum
from multiprocessing import Process, Pipe

# Message header id's for messages passed between the application
# process and the microcontroller and video processes

TinyOpD = {
    'setPwmOp':     0,
    'noteOnVelOp':  1,
    'noteOnUsecOp': 2,
    'noteOffOp':    3,
    'readOp':       4,
    'writeOp':      5
}

class TinyRegAddr(Enum):
    kRdRegAddrAddr   =  0,
    kRdTableAddrAddr =  1,
    kRdEEAddrAddr    =  2,
    kRdSrcAddr       =  3,
    kWrRegAddrAddr   =  4,
    kWrTableAddrAddr =  5,
    kWrEEAddrAddr    =  6,
    kWrDstAddr       =  7,
    kTmrCoarseAddr   =  8,
    kTmrFineAddr     =  9,
    kTmrPrescaleAddr = 10, 
    kPwmEnableAddr   = 11,
    kPwmDutyAddr     = 12,
    kPwmFreqAddr     = 13,
    kModeAddr        = 14,
    kStateAddr       = 15,
    kErrorCodeAddr   = 16

class TinyConst(Enum):
    kRdRegSrcId    = TinyRegAddr.kRdRegAddrAddr,    # 0
    kRdTableSrcId  = TinyRegAddr.kRdTableAddrAddr,  # 1
    kRdEESrcId     = TinyRegAddr.kRdEEAddrAddr      # 2
    
    kWrRegDstId    = TinyRegAddr.kWrRegAddrAddr,    # 4
    kWrTableDstId  = TinyRegAddr.kWrTableAddrAddr,  # 5
    kWrEEDstId     = TinyRegAddr.kWrEEAddrAddr,     # 6
    kWrAddrFl      = 0x08,                          # first avail bit above kWrEEAddr
    
class SerialMsgId(Enum):
    QUIT_MSG   = 0xffff   
    DATA_MSG   = 0xfffe


def _serial_process_func( serial_dev, baud, pipe ):

    reset_N     = 0
    drop_N      = 0
    noSync_N    = 0
    
    with serial.Serial(serial_dev, baud) as port:

        while True:

            # get the count of available bytes in the serial port buffer
            bytes_waiting_N = port.in_waiting


            # if no serial port bytes are available then sleep ....
            if bytes_waiting_N == 0:                    
                time.sleep(0.01)  # ... for 10 ms

            else: # read the serial port ...
                v = port.read(bytes_waiting_N)

                pipe.send((SerialMsgId.DATA_MSG,v)) # ... and send it to the parent
                            
            
            msg = None
            if pipe.poll():  # non-blocking check for parent process messages
                try:
                    msg = pipe.recv()
                except EOFError:
                    break
                
            # if an incoming message was received
            if msg != None:

                # this is a shutdown msg
                if msg[0] == SerialMsgId.QUIT_MSG:
                    pipe.send(msg) # ... send quit msg back
                    break

                # this is a data xmit msg
                elif msg[0] == SerialMsgId.DATA_MSG:
                    port.write(msg[1])
                




class SerialProcess(Process):
    def __init__(self,serial_dev,serial_baud):
        self.parent_end, child_end = Pipe()
        super(SerialProcess, self).__init__(target=_serial_process_func, name="Serial", args=(serial_dev,serial_baud,child_end,))
        self.doneFl    = False
        
    def quit(self):
        # send quit msg to the child process
        self.parent_end.send((SerialMsgId.QUIT_MSG,0))

    def send(self,msg_id,value):
        # send a msg to the child process
        self.parent_end.send((msg_id,value))

    def recv(self):
        # 
        
        x = None
        if not self.doneFl and self.parent_end.poll():
            x = self.parent_end.recv()
            
            if x[0] == SerialMsgId.QUIT_MSG:
                self.doneFl = True
            
        return x
    
    def is_done(self):
        return self.doneFl

        

class Picadae:
    def __init__( self, key_mapL, i2c_base_addr=1, serial_dev='/dev/ttyACM0', serial_baud=38400 ):
        """
        key_mapL      = [{ index, board, ch, type, midi, class }]  
        serial_dev    = /dev/ttyACM0
        serial_baud   = 38400
        i2c_base_addr = 1
        """        
        self.serialProc     = SerialProcess( serial_dev, serial_baud )
        self.keyMapD        = { d['midi']:d for d in key_mapL }
        self.i2c_base_addr  = i2c_base_addr
        self.prescaler_usec = 32
        
        self.serialProc.start()
        
    def close( self ):
        self.serialProc.quit()
        
    def write( self, i2c_addr, reg_addr, byteL ):
        return self._send( 'w', i2c_addr, reg_addr, [ len(byteL) ] + byteL )

    def set_read_addr( self, i2c_addr, src, addr ):
        return self. write(i2c_addr, TinyOpD['readOp'], src, addr )
            
    def set_reg_read_addr( self, i2c_addr, addr ):
        return self.set_read_addr(i2c_addr, TinyRegAddr.kRdRegAddrAddr, addr )

    def set_table_read_addr( self, i2c_addr, addr ):
        return self.set_read_addr(i2c_addr, TinyRegAddr.kRdTableAddrAddr, addr )
    
    def set_eeprom_read_addr( self, i2c_addr, addr ):
        return self.set_read_addr(i2c_addr, TinyRegAddr.kRdEEAddrAddr, addr )
    
    def read_request( self, i2c_addr, reg_addr, argL ):
        return self._send( 'r', i2c_addr, reg_addr,    [  byteOutN, len(argL) ] + argL )

    def read_poll( self ):
        return self.serialProc.recv()

    def read_block( self, i2c_addr, reg_addr, argL, byteOutN, time_out_ms ):
        
        self.read_request( self, i2c_addr, reg_addr, argL, byteOutN )

        ts = datetime.datetime.now() + datetime.timeDelta(milliseconds=time_out_ms)

        retL = []
        while datetime.datetime.now() < ts and len(retL) < byteOutN:
            
            x = self.serialProc.recv()
            if x is not None:
                retL.append(x)
            
            time.sleep(0.01)
        
        return retL
        

    def note_on_vel( self, midi_pitch, midi_vel ):
        return self.write( self._pitch_to_i2c_addr( midi_pitch ),
                           TinyOpD['noteOnVelOp'],
                           [self._validate_vel(midi_vel)] )
    
    def note_on_us( self, midi_pitch, pulse_usec ):
        return self.write( self._pitch_to_i2c_addr( midi_pitch ),
                           TinyOpD['noteOnUsecOp'],
                           list(self._usec_to_coarse_and_fine(pulse_usec)) )

    def note_off( self, midi_pitch ):
        return self.write( self._pitch_to_i2c_addr( midi_pitch ),
                           TinyOpD['noteOffOp'],[0] )  # TODO: sending a dummy byte because we can handle sending a command with no data bytes.

    def set_velocity_map( self, midi_pitch, midi_vel, pulse_usec ):
        pass
    
    def get_velocity_map( self, midi_pitch, midi_vel, time_out_ms=250 ):
        pass
    
    def set_pwm_duty( self, midi_pitch, duty_cycle_pct, enableFl=True ):
        return self.write( self._pitch_to_i2c_addr( midi_pitch ),
                           TinyOpD['setPwmOp'],
                           [enableFl, int( duty_cycle_pct * 255.0 /100.0 )])

    def get_pwm_duty( self, midi_pitch, time_out_ms=250 ):
        return self.read_block( self._pitch_to_i2c_addr( midi_pitch ),
                                TinyRegAddr.kPwmDutyAddr,
                                [], 1, time_out_ms )
    
    def set_pwm_freq( self, midi_pitch, freq_div_id ):
        # pwm frequency divider 1=1,2=8,3=64,4=256,5=1024
        assert( 1 <= freq_div_id and freq_div_id <= 5 )
        pass
    
    def get_pwm_freq( self, midi_pitch, time_out_ms=250 ):
        return self.read_block( self._pitch_to_i2c_addr( midi_pitch ),
                                TinyRegAddr.kPwmFreqAddr,
                                [], 1, time_out_ms )

    def set_flags( self, midi_pitch, flags ):
        return self.write( self._pitch_to_i2c_addr( midi_pitch ),                           
                           TinyOpD['writeOp'],
                           [ 12, 14, flags ])

    def make_note( self, midi_pitch, atk_us, dur_ms ):
        # TODO: handle error on note_on_us()
        self.note_on_us(midi_pitch, atk_us);
        time.sleep( dur_ms / 1000.0 )
        return self.note_off(midi_pitch)
        
    def _pitch_to_i2c_addr( self, pitch ):
        return self.keyMapD[ pitch ]['index'] + self.i2c_base_addr

    def _validate_vel( self, vel ):
        return vel

    def _usec_to_coarse_and_fine( self, usec ):
        
        coarse = int( usec / (self.prescaler_usec*255))
        fine   = int((usec - coarse*self.prescaler_usec*255) / self.prescaler_usec)

        print(coarse,fine)
        assert( coarse <= 255 )
        assert( fine <= 255)

        return coarse,fine

    def _send( self, opcode, i2c_addr, reg_addr, byteL  ):

        self._print( opcode, i2c_addr, reg_addr, byteL )
        
        byteA = bytearray( [ord(opcode), i2c_addr, reg_addr ] + byteL )

        return self.serialProc.send(SerialMsgId.DATA_MSG, byteA )

    def _print( self, opcode, i2c_addr, reg_addr, byteL ):
        
        s = "{} {} {}".format( opcode, i2c_addr, reg_addr )

        for x in byteL:
            s += " {}".format(x)


        print(s)

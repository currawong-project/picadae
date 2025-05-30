##| Copyright: (C) 2018-2020 Kevin Larke <contact AT larke DOT org>
##| License: GNU GPL version 3.0 or above. See the accompanying LICENSE file.

import os,sys,argparse,yaml,types,select,serial,logging,time,datetime
from enum import Enum
from multiprocessing import Process, Pipe

# Message header id's for messages passed between the application
# process and the microcontroller and video processes

class TinyOp(Enum):
    setPwmOp     = 0
    noteOnVelOp  = 1
    noteOnUsecOp = 2
    noteOffOp    = 3
    setReadAddr  = 4
    writeOp      = 5
    writeTableOp = 6
    holdDelayOp  = 7
    flagsOp      = 8
    invalidOp    = 9
    

class TinyRegAddr(Enum):
    kRdRegAddrAddr   =  0
    kRdTableAddrAddr =  1
    kRdEEAddrAddr    =  2
    kRdSrcAddr       =  3
    kWrRegAddrAddr   =  4
    kWrTableAddrAddr =  5
    kWrEEAddrAddr    =  6
    kWrDstAddr       =  7
    kTmrCoarseAddr   =  8
    kTmrFineAddr     =  9
    kTmrPrescaleAddr = 10 
    kPwmDutyAddr     = 11
    kPwmFreqAddr     = 12
    kPwmDivAddr      = 13
    kStateAddr       = 14
    kErrorCodeAddr   = 15
    kMaxAllowTmrAddr = 16
    kDelayCoarseAddr = 17
    kDelayFineAddr   = 18
    kFlagsAddr       = 19
    
class TinyRegAddr0(Enum):
    kRdRegAddrAddr   =  0
    kRdSrcAddr       =  1
    kWrRegAddrAddr   =  2
    kWrDstAddr       =  3
    kAttkDutyAddr    =  4
    kAttkDurHiAddr   =  5
    kAttkDurLoAddr   =  6
    kDecayStepAddr   =  7
    kDecayDecrAddr   =  8
    kPwmDutyAddr     =  9
    kErrorCodeAddr   = 10

    
class TinyConst(Enum):
    kRdRegSrcId    = TinyRegAddr.kRdRegAddrAddr.value    # 0
    kRdTableSrcId  = TinyRegAddr.kRdTableAddrAddr.value  # 1
    kRdEESrcId     = TinyRegAddr.kRdEEAddrAddr.value     # 2
    
    kWrRegDstId    = TinyRegAddr.kWrRegAddrAddr.value    # 4
    kWrTableDstId  = TinyRegAddr.kWrTableAddrAddr.value  # 5
    kWrEEDstId     = TinyRegAddr.kWrEEAddrAddr.value     # 6
    kWrAddrFl      = 0x08                                # first avail bit above kWrEEAddr
    
class SerialMsgId(Enum):
    QUIT_MSG   = 0xffff   
    DATA_MSG   = 0xfffe

class Result(object):
    def __init__( self, value=None, msg=None ):
        self.value = value
        self.msg   = msg

    def set_error( self, msg ):
        if self.msg is None:
            self.msg = ""
            
        self.msg += " " + msg
        
    def __bool__( self ):
        return self.msg is  None

    
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
        return Result()

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
    def __init__( self, key_mapL, i2c_base_addr=21, serial_dev='/dev/ttyACM0', serial_baud=38400, prescaler_usec=16 ):
        """
        key_mapL      = [{ index, board, ch, type, midi, class }]  
        serial_dev    = /dev/ttyACM0
        serial_baud   = 38400
        i2c_base_addr = 1
        """        
        self.serialProc     = SerialProcess( serial_dev, serial_baud )
        self.keyMapD        = { d['midi']:d for d in key_mapL }
        self.i2c_base_addr  = i2c_base_addr
        self.prescaler_usec = prescaler_usec
        self.log_level      = 0
        
        self.serialProc.start()

    def close( self ):
        self.serialProc.quit()
        
    def wait_for_serial_sync(self, timeoutMs=10000):

        # wait for the letter 'a' to come back from the serial port
        result = self.block_on_serial_read(1,timeoutMs)

        if result and len(result.value)>0 and result.value[0] == ord('a'):
            pass
        else:
            result.set_error("Serial sync failed.")

        return result
        
    def write_tiny_reg( self, i2c_addr, reg_addr, byteL ):
        return self._send( 'w', i2c_addr, reg_addr, [ len(byteL) ] + byteL )

    def call_op( self, midi_pitch, op_code, argL ):
        return self.write_tiny_reg( self._pitch_to_i2c_addr( midi_pitch ), op_code, argL )                          

    def set_read_addr( self, i2c_addr, mem_id, addr ):
        # mem_id: 0=reg_array 1=vel_table 2=eeprom
        return self.write_tiny_reg(i2c_addr, TinyOp.setReadAddr.value,[ mem_id, addr ])
                
    def read_request( self, i2c_addr, reg_addr, byteOutN ):
        return self._send( 'r', i2c_addr, reg_addr,[ byteOutN ] )

    def block_on_serial_read( self, byteOutN, time_out_ms=250 ):
        
        ts   = datetime.datetime.now() + datetime.timedelta(milliseconds=time_out_ms)
        retL = []
        
        while datetime.datetime.now() < ts and len(retL) < byteOutN:

            # If a value is available at the serial port return is otherwise return None.
            x = self.serialProc.recv()
            
            if x is not None and x[0] == SerialMsgId.DATA_MSG:
                for b in x[1]:
                    retL.append(int(b))
            
            time.sleep(0.01)

        result = Result(value=retL)
            
        if len(retL) < byteOutN:
            result.set_error("Serial port time out on read.")

        return result
            
            

    def block_on_picadae_read( self, midi_pitch, mem_id, reg_addr, byteOutN, time_out_ms=250 ):

        i2c_addr = self._pitch_to_i2c_addr( midi_pitch )
        
        result   = self.set_read_addr( i2c_addr, mem_id, reg_addr )

        if result:
            result = self.read_request( i2c_addr, TinyOp.setReadAddr.value, byteOutN )

            if result:
                result = self.block_on_serial_read( byteOutN, time_out_ms )
                
        return result


    def block_on_picadae_read_reg( self, midi_pitch, reg_addr, byteOutN=1, time_out_ms=250 ):
        return self.block_on_picadae_read( midi_pitch,
                                           TinyRegAddr.kRdRegAddrAddr.value,
                                           reg_addr,
                                           byteOutN,
                                           time_out_ms )

    def note_on_vel( self, midi_pitch, midi_vel ):
        return  self.call_op( midi_pitch, TinyOp.noteOnVelOp.value, [self._validate_vel(midi_vel)] )
        
    def note_on_us( self, midi_pitch, pulse_usec ):
        return  self.call_op( midi_pitch, TinyOp.noteOnUsecOp.value, list(self._usec_to_coarse_and_fine(pulse_usec)) )

    def note_off( self, midi_pitch ):
        return self.call_op( midi_pitch, TinyOp.noteOffOp.value,
                           [0] )  # TODO: sending a dummy byte because we can't handle sending a command with no data bytes.

    def set_hold_delay( self, midi_pitch, pulse_usec ):
        return  self.call_op( midi_pitch, TinyOp.holdDelayOp.value, list(self._usec_to_coarse_and_fine(pulse_usec)) )

    def get_hold_delay( self, midi_pitch, time_out_ms=250 ):

        res = self.block_on_picadae_read_reg( midi_pitch, TinyRegAddr.kDelayCoarseAddr.value, byteOutN=2, time_out_ms=time_out_ms )

        if len(res.value) == 2:
            res.value = [ self.prescaler_usec*255*res.value[0] + self.prescaler_usec*res.value[1] ]
        
        return res
        
    def set_flags( self, midi_pitch, flags ):
        return  self.call_op( midi_pitch, TinyOp.flagsOp.value, [int(flags)] )
        #return  self.call_op( midi_pitch, 5, [int(flags)] )

    def get_flags( self, midi_pitch, time_out_ms=250 ):        
         return self.block_on_picadae_read_reg( midi_pitch, TinyRegAddr.kFlagsAddr.value, byteOutN=1, time_out_ms=time_out_ms )
         #return self.block_on_picadae_read_reg( midi_pitch, TinyRegAddr.kAttkDutyAddr.value, byteOutN=1, time_out_ms=time_out_ms )
     
    
    def set_velocity_map( self, midi_pitch, midi_vel, pulse_usec ):
        coarse,fine = self._usec_to_coarse_and_fine( pulse_usec )
        src         = TinyConst.kWrAddrFl.value | TinyConst.kWrTableDstId.value
        addr        = midi_vel*2
        return self.call_op( midi_pitch, TinyOp.writeOp.value, [ src, addr, coarse, fine ] )
    
    def get_velocity_map( self, midi_pitch, midi_vel, time_out_ms=250 ):
        byteOutN = 2
        return self.block_on_picadae_read( midi_pitch, TinyConst.kRdTableSrcId.value, midi_vel*2, byteOutN, time_out_ms )

    def set_pwm_duty( self, midi_pitch, duty_cycle_pct ):
        if 0 <= duty_cycle_pct and duty_cycle_pct <= 100:
            # duty_cycle_pct = 100.0 - duty_cycle_pct
            return self.call_op( midi_pitch, TinyOp.setPwmOp.value, [ int( duty_cycle_pct * 255.0 /100.0 )])
        else:
            return Result(msg="Duty cycle (%f) out of range 0-100." % (duty_cycle_pct))

    def get_pwm_duty( self, midi_pitch, time_out_ms=250 ):
        return self.block_on_picadae_read_reg( midi_pitch, TinyRegAddr.kPwmDutyAddr.value, time_out_ms=time_out_ms )
    
    def set_pwm_freq( self, midi_pitch, freq ):
        res = self.get_pwm_duty( midi_pitch )
        if res:
            print("duty",int(res.value[0]))
            res = self.call_op( midi_pitch, TinyOp.setPwmOp.value, [ int(res.value[0]), int(freq) ])
        return res
    
    def get_pwm_freq( self, midi_pitch, time_out_ms=250 ):
        return self.block_on_picadae_read_reg( midi_pitch, TinyRegAddr.kPwmFreqAddr.value, time_out_ms=time_out_ms )

    def get_pwm_div( self, midi_pitch, time_out_ms=250 ):
        return self.block_on_picadae_read_reg( midi_pitch, TinyRegAddr.kPwmDivAddr.value, time_out_ms=time_out_ms )

    def set_pwm_div( self, midi_pitch, div, time_out_ms=250 ):
        res = self.get_pwm_duty( midi_pitch )
        if res:
            duty = res.value[0]
            res = self.get_pwm_freq( midi_pitch )
            if res:
                res = self.call_op( midi_pitch, TinyOp.setPwmOp.value, [ int(duty), int(res.value[0]), int(div) ])
        return res
    
    def write_table( self, midi_pitch, time_out_ms=250 ):
        # TODO: sending a dummy byte because we can't handle sending a command with no data bytes.
        return self.call_op( midi_pitch, TinyOp.writeTableOp.value,[0])

    def make_note( self, midi_pitch, atk_us, dur_ms ):
        # TODO: handle error on note_on_us()
        self.note_on_us(midi_pitch, atk_us);
        time.sleep( dur_ms / 1000.0 )
        return self.note_off(midi_pitch)

    def make_seq( self, midi_pitch, base_atk_us, dur_ms, delta_us, note_cnt ):
        for i in range(note_cnt):
            self.make_note( midi_pitch, base_atk_us + i*delta_us, dur_ms )
            time.sleep( dur_ms / 1000.0 )
        return Result()
    
    def make_scale( self, pitch0, pitch1, atk_us, dur_ms ):

        if pitch0>pitch1:
            printf("pitch0 must be <= pitch1")
        else:
            for pitch in range(pitch0,pitch1+1):            
                self.make_note( pitch, atk_us, dur_ms )
                time.sleep( dur_ms / 1000.0 )
            return Result()
            
    
    def set_log_level( self, log_level ):
        self.log_level = log_level
        return Result()
        
    def _pitch_to_i2c_addr( self, pitch ):
        return self.keyMapD[ pitch ]['index'] + self.i2c_base_addr

    def _validate_vel( self, vel ):
        return vel

    def _usec_to_coarse_and_fine( self, usec ):

        coarse_usec = self.prescaler_usec*255 # usec's in one coarse tick
        
        coarse = int( usec / coarse_usec )
        fine   = int(round((usec - coarse*coarse_usec) / self.prescaler_usec))

        assert( coarse <= 255 )
        assert( fine <= 255)

        x = coarse*coarse_usec + fine*self.prescaler_usec
        


        ##### 
        # n = int(16e6*usec/(256*1e6))

        # coarse = n >> 8;
        # fine   = n & 0xff;

        # x = usec
        ####

        print("C:%i F:%i : %i %i (%i)" % (coarse,fine, x, usec, usec-x ))

        return coarse,fine

    def _send( self, opcode, i2c_addr, reg_addr, byteL  ):

        self._print( opcode, i2c_addr, reg_addr, byteL )
        
        byteA = bytearray( [ord(opcode), i2c_addr, reg_addr ] + byteL )

        return self.serialProc.send(SerialMsgId.DATA_MSG, byteA )

    def _print( self, opcode, i2c_addr, reg_addr, byteL ):

        if self.log_level:
            s = "{} {} {}".format( opcode, i2c_addr, reg_addr )

            for x in byteL:
                s += " {}".format(x)


            print(s)

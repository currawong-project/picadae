##| Copyright: (C) 2018-2020 Kevin Larke <contact AT larke DOT org>
##| License: GNU GPL version 3.0 or above. See the accompanying LICENSE file.

import os,sys,argparse,yaml,types,select,serial,logging,time

from multiprocessing import Process, Pipe

# Message header id's for messages passed between the application
# process and the microcontroller and video processes
QUIT_MSG   = 0xffff   
DATA_MSG   = 0xfffe
ERROR_MSG  = 0xfffd


def _reset_port(port):
    port.reset_input_buffer()
    port.reset_output_buffer()
    port.send_break()
    #self.reportResetFl  = True
    #self.reportStatusFl = True

def _serial_process_func( serial_dev, baud, sensor_count, pipe ):

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

                pipe.send((DATA_MSG,v)) # ... and send it to the parent

                            
            
            msg = None
            if pipe.poll():  # non-blocking check for parent process messages
                try:
                    msg = pipe.recv()
                except EOFError:
                    break
                
            # if an incoming message was received
            if msg != None:

                # this is a shutdown msg
                if msg[0] == QUIT_MSG:
                    pipe.send(msg) # ... send quit msg back
                    break

                # this is a data xmit msg
                elif msg[0] == DATA_MSG:
                    port.write(msg[1])
                




class SessionProcess(Process):
    def __init__(self,target,name,args=()):
        self.parent_end, child_end = Pipe()
        super(SessionProcess, self).__init__(target=target,name=name,args=args + (child_end,))
        self.doneFl    = False
        
    def quit(self):
        # send quit msg to the child process
        self.parent_end.send((QUIT_MSG,0))

    def send(self,msg_id,value):
        # send a msg to the child process
        self.parent_end.send((msg_id,value))

    def recv(self):
        x = None
        if not self.doneFl and self.parent_end.poll():
            x = self.parent_end.recv()
            
            if x[0] == QUIT_MSG:
                self.doneFl = True
            
        return x
    
    def isDone(self):
        return self.doneFl

class SerialProcess(SessionProcess):
    def __init__(self,serial_device,baud,foo):
        super(SerialProcess, self).__init__(_serial_process_func,"Serial",args=(serial_device,baud,foo))

    

class App:
    def __init__( self, cfg ):

        self.cfg       = cfg
        self.serialProc  = SerialProcess(cfg.serial_dev,cfg.serial_baud,0)


    def _update( self, quittingFl=False ):

        if self.serialProc.isDone():
            return False

        while True:
            msg = serialProc.recv()

            # no serial msg's were received
            if msg is None:
                break

            if msg[0] == DATA_MSG:
                print("in:",msg[1])


    def _parse_error( self, msg, cmd_str=None ):

        if cmd_str:
            msg += " Command:{}".format(cmd_str)
            
        return (None,msg)

    def _parse_int( self, token, var_label, min_value, max_value ):
        # convert the i2c destination address to an integer
        try:
            int_value = int(token)
        except ValueError:
            return self._parse_error("Synax error: '{}' is not a legal integer.".format(token))

        # validate the i2c address value
        if min_value > int_value or int_value > max_value:
            return self._parse_error("Syntax error: '{}' {} out of range 0 to {}.".format(token,int_value,max_value))

        return (int_value,None)
    
    def parse_app_cmd( self, cmd_str ):
        """
        Command syntax <opcode> <remote_i2c_addr> <value>
        """
        
        op_tok_idx  = 0
        i2c_tok_idx = 1
        val_tok_idx = 2
        
        cmdD = {
            'p':{ 'reg':0, 'n':1, 'min':0, 'max':4 },       # timer pre-scalar: sets timer tick rate
            't':{ 'reg':1, 'n':2, 'min':0, 'max':10e7 },    # microseconds
            'd':{ 'reg':3, 'n':1, 'min':0, 'max':100 },     # pwm duty cylce (0-100%)
            'f':{ 'reg':4, 'n':1, 'min':1, 'max':5 },       # pwm frequency divider 1=1,2=8,3=64,4=256,5=1024
            }

        cmd_str = cmd_str.strip()

        tokenL = cmd_str.split(' ')

        # validate the counf of tokens
        if len(tokenL) != 3:
            return self._parse_error("Syntax error: Invalid token count.",cmd_str)

        opcode = tokenL[op_tok_idx]
        
        # validate the opcode
        if opcode not in cmdD:
            return self._parse_error("Syntax error: Invalid opcode.",cmd_str)

        # convert the i2c destination address to an integer
        i2c_addr, msg = self._parse_int( tokenL[i2c_tok_idx], "i2c address", 0,127 )

        if i2c_addr is None:
            return (None,msg)

        d = cmdD[ opcode ]
        
        # get the value
        value, msg = self._parse_int( tokenL[val_tok_idx], "command value", d['min'], d['max'] )

        if value is None:
            return (value,msg)

        dataL = [ value ]

        if opcode == 't':
            
            coarse = int(value/(32*254))
            fine   = int((value - coarse*32*254)/32)
            print(coarse,fine)
            dataL  = [ coarse, fine ]

        elif opcode == 'd':
            dataL = [ int(value * 255 / 100.0) ]

        cmd_bV = bytearray( [ ord('w'), i2c_addr, d['reg'], len(dataL) ] + dataL )

        if False:
            print('cmd_bV:')
            for x in cmd_bV:
                print(int(x))

        return (cmd_bV,None)
    
    def parse_cmd( self, cmd_str ):

        op_tok_idx  = 0
        i2c_tok_idx = 1
        reg_tok_idx = 2
        rdn_tok_idx = 3
        
        cmd_str = cmd_str.strip()

        # if this is a high level command
        if cmd_str[0] not in ['r','w']:
            return self.parse_app_cmd( cmd_str )

        # convert the command string to tokens
        tokenL = cmd_str.split(' ')

        # no commands require fewer than 4 tokens
        if len(tokenL) < 4:
            return self._parse_error("Syntax error: Missing tokens.")

        # get the command opcode
        op_code = tokenL[ op_tok_idx ]

        # validate the opcode
        if op_code not in [ 'r','w']:
            return self._parse_error("Unrecognized opcode: {}".format( op_code ))

        # validate the token count given the opcode
        if op_code == 'r' and len(tokenL) != 4:
            return self._parse_error("Syntax error: Illegal read syntax.")

        if op_code == 'w' and len(tokenL) < 4:
            return self._parse_error("Syntax error: Illegal write command too short.")

        # convert the i2c destination address to an integer
        i2c_addr, msg = self._parse_int( tokenL[i2c_tok_idx], "i2c address", 0,127 )

        if i2c_addr is None:
            return (None,msg)

        reg_addr, msg = self._parse_int( tokenL[reg_tok_idx], "reg address", 0, 255 )

        if reg_addr is None:
            return (None,msg)

        dataL      = []

        # parse and validate the count of bytes to read
        if op_code == 'r':
            op_byteN, msg = self._parse_int( tokenL[ rdn_tok_idx ], "read byte count", 0, 255 )

            if op_byteN is None:
                return (None,msg)

            
        # parse and validate the values to write    
        elif op_code == 'w':

            for j,i in enumerate(range(reg_tok_idx+1,len(tokenL))):
                value, msg = self._parse_int( tokenL[i], "write value: %i" % (j), 0, 255 )
                
                if value is None:
                    return (None,msg)
                
                dataL.append(value)
                
            op_byteN = len(dataL)
            
        # form the command into a byte array
        cmd_bV = bytearray( [ ord(op_code), i2c_addr, reg_addr, op_byteN ] + dataL )

        # s = ""
        # for i in range(len(cmd_bV)):
        #    s += "%i " % (cmd_bV[i])
        # print(s)
        
        return (cmd_bV,None)

        
    def run( self ):

        self.serialProc.start()
        
        print("'quit' to exit")
        time_out_secs = 1
        
        while True:
            
            i, o, e = select.select( [sys.stdin], [], [], time_out_secs )

            if (i):
                s = sys.stdin.readline().strip()
                
                if s == 'quit' or s == 'q':
                    break
                
                cmd_bV,err_msg = self.parse_cmd(s)

                if cmd_bV is None:
                    print(err_msg)
                else:
                    self.serialProc.send( DATA_MSG, cmd_bV )
                
                
            else:
                # wait timed out
                msg = self.serialProc.recv()

                # if a serial msg was received
                if msg is not None and msg[0] == DATA_MSG:
                    str = ""
                    for i in range(len(msg[1])):
                        str += "{} ".format(int(msg[1][i]))
                        
                    print("ser:",str)
            
            
        self.serialProc.quit()

def parse_args():
    """Parse the command line arguments."""
    
    descStr  = """Picadae auto-calibrate."""
    logL     = ['debug','info','warning','error','critical']
    
    ap = argparse.ArgumentParser(description=descStr)


    ap.add_argument("-s","--setup",                     default="picadae_cmd.yml",  help="YAML configuration file.")
    ap.add_argument("-c","--cmd",   nargs="*",                                   help="Give a command as multiple tokens")
    ap.add_argument("-r","--run",                                                help="Run a named command list from the setup file.")
    ap.add_argument("-l","--log_level",choices=logL,     default="warning",      help="Set logging level: debug,info,warning,error,critical. Default:warning")

    return ap.parse_args()
    
            
def parse_yaml_cfg( fn ):
    """Parse the YAML configuration file."""
    
    cfg  = None
    
    with open(fn,"r") as f:
        cfgD = yaml.load(f, Loader=yaml.FullLoader)

        cfg = types.SimpleNamespace(**cfgD['picadae_cmd'])

    return cfg



if __name__ == "__main__":

    args = parse_args()

    cfg = parse_yaml_cfg( args.setup )

    app = App(cfg)

    app.run()

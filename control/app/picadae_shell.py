##| Copyright: (C) 2018-2020 Kevin Larke <contact AT larke DOT org>
##| License: GNU GPL version 3.0 or above. See the accompanying LICENSE file.

import os,sys,argparse,yaml,types,select,serial,logging,time

from picadae_api import Picadae
from picadae_api import Result

class PicadaeShell:
    def __init__( self, cfg ):
        self.p      = None
        self.parseD = {
            'q':{ "func":None,             "minN":0,  "maxN":0, "help":"quit"},
            '?':{ "func":"_help",          "minN":0,  "maxN":0, "help":"Print usage text."},
            'w':{ "func":"_write",         "minN":-1, "maxN":-1,"help":"write <i2c_addr> <reg_addr> <data0> ... <dataN>"},
            'r':{ "func":"_read",          "minN":4,  "maxN":4, "help":"read  <i2c_addr> <src> <reg_addr> <byteN>   src: 0=reg_array 1=vel_table 2=eeprom"},
            'v':{ "func":"note_on_vel",    "minN":2,  "maxN":2, "help":"note-on <pitch> <vel>"},
            'u':{ "func":"note_on_us",     "minN":2,  "maxN":3, "help":"note-on <pitch> <usec> <prescale> (1=1, 2=8 .5us, 3=64 4us,(4)=256 16us, 5=1024 64us)"},
            'o':{ "func":"note_off",       "minN":1,  "maxN":1, "help":"note-off <pitch>"},
            'T':{ "func":"set_vel_map",    "minN":3,  "maxN":3, "help":"table <pitch> <vel> <usec>"},
            't':{ "func":"get_vel_map",    "minN":2,  "maxN":2, "help":"table <pitch> <vel>"},
            'D':{ "func":"set_pwm_duty",   "minN":2,  "maxN":4, "help":"duty <pitch> <percent> {<hz> {<div>}} " },
            'd':{ "func":"get_pwm_duty",   "minN":1,  "maxN":1, "help":"duty <pitch>"},
            'H':{ "func":"set_hold_delay", "minN":2,  "maxN":2, "help":"hold delay <pitch> <usec>"},
            'h':{ "func":"get_hold_delay", "minN":1,  "maxN":1, "help":"hold delay <pitch>"},
            'F':{ "func":"set_pwm_freq",   "minN":2,  "maxN":2, "help":"pwm freq <pitch> <hz> 254=~123Hz"},
            'f':{ "func":"get_pwm_freq",   "minN":1,  "maxN":1, "help":"pwm freq <pitch>"},
            'I':{ "func":"set_pwm_div",    "minN":2,  "maxN":2, "help":"pwm div <pitch> <div> div:2=2,3=4,4=8,(5)=16 1us,6=32,7=64,8=128,9=256,10=512 32us, 11=1024,12=2048,13=4096,14=8192,15=16384"},
            'i':{ "func":"get_pwm_div",    "minN":1,  "maxN":1, "help":"pwm div <pitch>"},
            'W':{ "func":"write_table",    "minN":1,  "maxN":1, "help":"write_table <pitch>"},
            'N':{ "func":"make_note",      "minN":3,  "maxN":3, "help":"note <pitch> <atkUs> <durMs>"},
            'S':{ "func":"make_seq",       "minN":5,  "maxN":5, "help":"seq  <pitch> <atkUs> <durMs> <deltaUs> <note_count>"}, 
            'L':{ "func":"set_log_level",  "minN":1,  "maxN":1, "help":"log <level> (0-1)."}
            }

    def _help( self, _=None ):
        for k,d in self.parseD.items():
            s = "{} = {}".format( k, d['help'] )
            print(s)
        return Result()

    def _write( self, argL ):
        return self.p.write(argL[0], argL[1], argL[2:])
    
    def _read( self, i2c_addr, src_id, reg_addr, byteN ):
        return self.p.block_on_picadae_read(i2c_addr, src_id, reg_addr, byteN)
        
    def _syntaxError( self, msg ):
        print("Syntax Error: " + msg )
        return Result()
            
    def _exec_cmd( self, tokL ):

        result = Result()
        
        if len(tokL) <= 0:
            return None

        opcode = tokL[0]
        
        if opcode not in self.parseD:
            return self._syntaxError("Unknown opcode: '{}'.".format(opcode))

        d = self.parseD[ opcode ]

        func_name = d['func']
        func      = None

        # find the function associated with this command
        if hasattr(self, func_name ):
            func = getattr(self, func_name )
        elif hasattr(self.p, func_name ):
            func = getattr(self.p, func_name )
        else:
           return self._syntaxError("Exec function not found: '{}'.".format(func_name))

        try:
            # convert the parameter list into integers
            argL = [ int(tokL[i]) for i in range(1,len(tokL)) ]
        except:
            return self._syntaxError("Unable to create integer arguments.")

        # validate the count of command args
        if  d['minN'] != -1 and (d['minN'] > len(argL) or len(argL) > d['maxN']):                
            return self._syntaxError("Argument count mismatch. {} is out of range:{} to {}".format(len(argL),d['minN'],d['maxN']))

        # call the command function
        result = func(*argL)

        return result
    
    def run( self ):

        # create the API object
        self.p = Picadae( cfg.key_mapL, cfg.i2c_base_addr, cfg.serial_dev, cfg.serial_baud, cfg.prescaler_usec )

        # wait for the letter 'a' to come back from the serial port
        result = self.p.wait_for_serial_sync(timeoutMs=cfg.serial_sync_timeout_ms)

        if not result:
            print("Serial port sync failed.")
        else:
            print(result.value)
            
            print("'q'=quit '?'=help")
            time_out_secs = 1

            while True:

                # wait for keyboard activity
                i, o, e = select.select( [sys.stdin], [], [], time_out_secs )

                if (i):
                    # read the command
                    s = sys.stdin.readline().strip() 

                    # tokenize the command
                    tokL = s.split(' ')

                    # if this is the 'quit' command
                    if tokL[0] == 'q':
                        break

                    # execute the command
                    result = self._exec_cmd( tokL )

                    if result.value:
                        print(result.value)
                
                
        self.p.close()
    
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

    app = PicadaeShell(cfg)

    app.run()

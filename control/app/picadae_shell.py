import os,sys,argparse,yaml,types,select,serial,logging,time

from picadae_api import Picadae
from picadae_api import Result

class PicadaeShell:
    def __init__( self, cfg ):
        self.p      = None
        self.parseD = {
            'q':{ "func":None,           "varN":0,  "help":"quit"},
            '?':{ "func":"help",         "varN":0,  "help":"Print usage text."},
            'w':{ "func":"write",        "varN":-1, "help":"write <i2c_addr> <reg_addr> <data0> ... <dataN>"},
            'r':{ "func":"read",         "varN":3,  "help":"read  <i2c_addr> <reg_addr> <byteN>"},
            'v':{ "func":"note_on_vel",  "varN":2,  "help":"note-on <pitch> <vel>"},
            'u':{ "func":"note_on_us",   "varN":2,  "help":"note-on <pitch> <usec>"},
            'o':{ "func":"note_off",     "varN":1,  "help":"note-off <pitch>"},
            'T':{ "func":"set_vel_map",  "varN":3,  "help":"table <pitch> <vel> <usec>"},
            't':{ "func":"get_vel_map",  "varN":2,  "help":"table <pitch> <vel>"},
            'D':{ "func":"set_pwm_duty", "varN":2,  "help":"duty <pitch> <percent>"},
            'd':{ "func":"get_pwm_duty", "varN":1,  "help":"duty <pitch>"},
            'F':{ "func":"set_pwm_freq", "varN":2,  "help":"freq <pitch> <hz>"},
            'f':{ "func":"get_pwm_freq", "varN":1,  "help":"freq <pitch>"},
            'B':{ "func":"set_flags",    "varN":2,  "help":"flags <pitch> <flags>"},
            'N':{ "func":"make_note",    "varN":3,  "help":"note <pitch> atkUs durMs"},
            }

    def _do_help( self, _ ):
        for k,d in self.parseD.items():
            s = "{} = {}".format( k, d['help'] )
            print(s)
        return Result()

    def _do_write( self, argL ):
        return self.p.write(argL[0], argL[1], argL[2:])
    
    def _do_read( self, argL ):
        return self.p.read(*argL)

    def _do_note_on_vel( self, argL ):
        return self.p.note_on_vel(*argL)
    
    def _do_note_on_us( self, argL ):
        return self.p.note_on_us(*argL)

    def _do_note_off( self, argL ):
        return self.p.note_off(*argL)

    def _do_set_vel_map( self, argL ):
        return self.p.set_velocity_map(*argL)

    def _do_get_vel_map( self, argL ):
        return self.p.get_velocity_map(*argL)

    def _do_set_pwm_duty( self, argL ):
        return self.p.set_pwm_duty(*argL)

    def _do_get_pwm_duty( self, argL ):
        return self.p.get_pwm_duty(*argL)
    
    def _do_set_pwm_freq( self, argL ):
        return self.p.set_pwm_freq(*argL)

    def _do_get_pwm_freq( self, argL ):
        return self.p.get_pwm_freq(*argL)

    def _do_set_flags( self, argL ):
        return self.p.set_flags(*argL)

    def _do_make_note( self, argL ):
        return self.p.make_note(*argL)
        
    def _syntaxError( self, msg ):
        print("Syntax Error: " + msg )
        return Result()
            
    def _exec_cmd( self, tokL ):
        if len(tokL) <= 0:
            return None

        opcode = tokL[0]
        
        if opcode not in self.parseD:
            return self._syntaxError("Unknown opcode: '{}'.".format(opcode))

        d = self.parseD[ opcode ]

        func_name = "_do_" + d['func']

        if hasattr(self, func_name ):
            func   = getattr(self, func_name )

            try:
                argL = [ int(tokL[i]) for i in range(1,len(tokL)) ]
            except:
                return self._syntaxError("Unable to create integer arguments.")

            if  d['varN'] != -1 and len(argL) != d['varN']:                
                return self._syntaxError("Argument mismatch {} != {}.".format(len(argL),d['varN']))
            
            result = func(argL)
            

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

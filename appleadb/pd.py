import sigrokdecode as srd

adb_timings = {
    'global-reset': 3e-3,
    'attention': 800e-6 * (1-(3/100)),
    'sync-min': 65e-6 * (1-(10/100)),
    'sync-max': 65e-6 * (1+(10/100)),
    'stop-min': 65e-6 * (1-(30/100)) * (1-(5/100)),
    'stop-max': 65e-6 * (1+(30/100)) * (1+(5/100)),
}

class SamplerateError(Exception):
    pass

class Decoder(srd.Decoder):
    api_version=2
    id = 'appleadb'
    name = 'ADB'
    longname = 'Apple Desktop Bus'
    desc = 'Apple proprietary protocol for desktop peripherals'
    license = 'mit'
    inputs = ['logic']
    outputs = ['adb']
    channels = ({'id':'adb', 'name': 'ADB', 'desc': 'ADB data line'},)
    annotations = (
        ('bit','Bit'),
        ('reset', 'Reset'),
        ('attention', 'Attention'),
        ('sync', 'Sync'),
        ('tlt', 'Stop-to-start-Time'),
        ('srq', 'Service Request'),
        ('cmd', 'Command'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0, 1, 2, 3, 4, 5)),
        ('command', 'Command', (6,)),
    )

    def __init__(self):
        self.samplerate = None
        self.reset = False
        self.last_fe = None
        self.prev_last_fe = None
        self.last_re = None
        self.oldpin = None
        self.state = 'idle'
        self.duty = None
        self.command = 0
        self.command_start = 0
        self.command_bits = 0

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def time_last_fe(self):
        return (self.samplenum - self.last_fe)/self.samplerate

    def time_last_re(self):
        return (self.samplenum - self.last_re)/self.samplerate

    def last_fe_first(self):
        if self.last_fe is None or self.last_re is None:
            return True
        else:
            return self.last_fe > self.last_re

    def last_re_first(self):
        if self.last_fe is None or self.last_re is None:
            return True
        else:
            return self.last_re > self.last_fe

    def command_text(self):
        device_address = (self.command >> 4) & 0xF
        command_code = (self.command >> 2) & 0x3
        register_code = self.command & 0x3

        if register_code == 0 and command_code == 0:
            return "SendReset"
        elif command_code == 0 and register_code == 1:
            return "Flush (address: %d)" % device_address
        elif command_code == 0 and register_code > 1:
            return "Reserved"
        elif command_code == 1:
            return "Reserved"
        elif command_code == 2:
            return "Listen (address: %d, register: %d)" % (device_address, register_code)
        elif command_code == 3:
            return "Talk (address: %d, register: %d)" % (device_address, register_code)
        else:
            return "Unknown"

    def decode(self, es, ss, data):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        for (self.samplenum, (pin, )) in data:
            if self.oldpin is None:
                self.oldpin = pin
                continue
            
            # State machine
            if not self.reset and \
               not pin and \
               self.last_fe is not None and \
               self.time_last_fe() > adb_timings['global-reset'] and \
               self.last_fe_first():
                self.state = 'idle'
                self.command = 0
                self.command_bits = 0
                self.reset = True
                self.put(self.last_fe, self.samplenum, self.out_ann, [1, ['Reset', 'RST', 'R']])
            elif not self.reset and \
                 not pin and \
                 self.last_fe is None and \
                 (self.samplenum/self.samplerate) > adb_timings['global-reset']:
                self.state = 'idle'
                self.command = 0
                self.command_bits = 0
                self.reset = True
                self.put(0, self.samplenum, self.out_ann, [1, ['Reset', 'Rst', 'R']])
            elif not self.reset and \
                 pin and \
                 self.state == 'idle' and \
                 self.last_fe is not None and \
                 self.last_re is not None and \
                 self.time_last_fe() > adb_timings['attention'] and \
                 self.last_re == self.samplenum-1:
                self.state = 'attention'
                self.put(self.last_fe, self.samplenum-1, self.out_ann, [2, ['ATTENTION', 'ATT', 'A']])
            elif not self.reset and \
                 not pin and \
                 self.state == 'attention' and \
                 self.last_re is not None and \
                 self.last_fe is not None and \
                 self.time_last_re() > adb_timings['sync-min'] and \
                 self.last_fe == self.samplenum-1:
                self.state = 'sync'
                self.put(self.last_re, self.samplenum-1, self.out_ann, [3, ['SYNC', 'SYN', 'S']])
                self.command_start = self.samplenum-1
            elif not self.reset and \
                 self.state == 'sync' and \
                 self.prev_last_fe is not None and \
                 self.last_fe == self.samplenum-1 and \
                 self.command_bits < 8:
                if self.duty > 50:
                    self.put(self.prev_last_fe, self.last_fe, self.out_ann, [0, ['1']])
                    self.command += 1 << (7 - self.command_bits)
                else:
                    self.put(self.prev_last_fe, self.last_fe, self.out_ann, [0, ['0']])
                self.command_bits += 1
            elif not self.reset and \
                 self.state == 'sync' and \
                 self.command_bits == 8 and \
                 pin and \
                 self.time_last_fe() >= adb_timings['stop-min']:
                self.state = 'stop'
                self.put(self.last_fe, self.samplenum, self.out_ann, [1, ['STOP']])
                self.put(self.command_start, self.last_fe, self.out_ann, [6, [self.command_text()]])
            elif not self.reset and \
                 self.state == 'stop':
                self.state = 'stop-to-start'
                self.command = 0
                self.command_bits = 0
            elif not self.reset and \
                 self.state == 'stop-to-start' and \
                 self.time_last_fe() >= 300e-6:
                if not pin:
                    self.state = 'caca'
                else:
                    self.state = 'idle'

            # Edge detection
            if not self.oldpin and pin:
                self.reset = False
                self.last_re = self.samplenum
            elif self.oldpin and not pin:
                if self.last_fe and self.last_re:
                    self.duty = 100*(self.samplenum-self.last_re)/(self.samplenum - self.last_fe)

                if self.last_fe is not None:
                    self.prev_last_fe = self.last_fe
                self.last_fe = self.samplenum

            self.oldpin = pin

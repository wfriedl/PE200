#!/usr/bin/env python

"""
    Perkin Elmer interface for 200 Series Pump

    This interface was derived on a rain day in the lab when I needed a
    remotely controlled pump to do some experiments.  It was initially written
    while I was learning Python so be gentle.
"""
import serial
import time
import os
import sys


class LCPumpError(Exception):
    pass


class BadParam(LCPumpError):
    """ Error 3 """
    pass


class BadCmd(LCPumpError):
    """ Error 9 """
    pass


class MethodError(LCPumpError):
    pass


states = {
    0: "SHTDN",
    1: "",
    2: "EQUIL",
    3: "READY",
    4: "RUN01",
    5: "RUN02",
    6: "RUN03",
    7: "RUN04",
    8: "RUN05",
    9: "RUN06",
    10: "RUN07",
    11: "RUN08",
    12: "RUN09",
    13: "RUN10",
    14: "RUN11",
    15: "RUN12",
    16: "RUN13",
    17: "RUN14",
    18: "RUN15",
    19: "RUN16",
    20: "RUN17",
    21: "RUN18",
    22: "RUN19",
    24: "HLD",
    }


def find_port():
    istty = lambda x: 'tty' in x

    if sys.platform == 'darwin':
        isUSB = lambda x: 'usbserial' in x
    elif sys.platform == 'linux2':
        isUSB = lambda x: 'USB' in x

    root, dirs, fnames = os.walk('/dev/').next()
    dirs = filter(isUSB, filter(istty, fnames))
    return [root + d for d in dirs]


class LC200Q(object):
    """
        Perkin Elmer Quaternary LC Pump (200 Series)

        How to use this:

        ports = find_port()
        for dev in ports:
            if LC200Q.test(dev):
                ## Success use this device
                newDev=LC200Q(dev)

    """
    @staticmethod
    def test(port):
        """
            Test a tty device to see if it is a Compatible Device.
            return instance of serial.Serial on success or None on Failure
        """
        print 'testing: ', port
        if isinstance(port, str):
            ser = serial.Serial()
            ser.port = port
        else:
            ser = port
        ser.baudrate = 2400
        ser.stopbits = 1
        ser.timeout = .25
        try:
            ser.close()
            ser.open()
            ser.write('I\n')
            tmp = ser.readline()
            if tmp.find('Version') >= 0:
                print "Found: ", ser
                return ser
        except (serial.SerialException, OSError):
            pass

    def __init__(self, ser=None, min_pressure=0, max_pressure=300, rdy=999):
        if ser is None:
            ser = scan_devs()
        self.ser = ser
        self.minPressure = min
        self.maxPressure = max
        self.rdy = rdy

    def flow(self, rate, abcd_proportions):
        """ rate=%s abcd=%s """ % (rate, abcd_proportions)
        args = rate + tuple(abcd_proportions)
        steps = ['A,1.0,%.2f,%s,%s,%s,%s,0' % args]
        self._method(steps)
        self.start()

    def gradient(self, t, rate, initial_abcd, final_abcd, curve=1.0):
        args = (t, rate, initial_abcd, final_abcd, curve)
        """ t=%s rate=%s abcd0=%s abcd1=%s curve=%s""" % args
        curve = 1 if curve < 1 else 9 if curve > 9 else curve
        if self.state() is not 0:
            self.stop()
        c1_args = rate + tuple(initial_abcd)
        steps = ['A,1.0,%.2f,%s,%s,%s,%s,0' % c1_args]
        c2_args = (t, rate,) + tuple(final_abcd) + (10 * curve,)
        steps += ['A,%.2f,%.2f,%s,%s,%s,%s,%d' % c2_args]
        c3_args = (rate,) + (final_abcd)
        steps += ['A,999,%.2f,%s,%s,%s,%s,0' % c3_args]
        self._method(steps)
        self.start()
        self.cmd('j')

    def _method(self, steps=[], events=None, retry=True):
        """	steps[0]=[str(time,flow rate, A, B, C, D, curve)]
            events are NOT implemented
        """
        if len(steps) > 10:
            raise MethodError('Please, less than 10 steps in a method.')
        ret = ''
        cmd = self.cmd
        cmd_list = steps
        cmd_list += ['A,0.0,0.01,0.0,0.0,0.0,100.0,0'] * (10 - len(steps))
        cmd_list += ['B,%.1d,%.1d,%.1d' % (
            self.minPressure,
            self.maxPressure,
            self.rdy)]
        # C - Setup Stored Events
        # cmd('C',',time,trigger,type[G,E]')
        ret += cmd('P')	 # P - Seize control
        try:
            ret += '\n'.join([cmd(i[0], i[1:]) for i in cmd_list])
        except (BadCmd, BadParam) as e:
            print 'Error sending method. Feedback: \n', ret
            print 'Retrying...\n', e
            self.reset()
            if retry:
                self._method(steps, events, False)
            else:
                raise
        ret += cmd('t')
        ret += cmd('l')
        ret += cmd('H')
        return ret

    def cmd(self, cmd, data='', eol='\n', delay=.01, debug=False):
        if debug:
            print "sending: %s%s" % (cmd, data)
        self.ser.write(cmd + data + eol)
        self.lastCmd = cmd + data + eol
        time.sleep(delay)
        tmp = self.ser.readline().strip()
        tmp = tmp if tmp != "" else self.ser.readline().strip()
        if tmp == '9':
            self._flush()
            msg = "err: Invalid Command for current state\n"
            msg += "Command= %s%s%s"
            raise BadCmd(msg % (cmd, data, eol))
        elif tmp == '3' and cmd not in ('a', 'c'):
            self._flush()
            msg = "err: Invalid Parameter\n"
            msg += "Command= %s%s%s"
            raise BadCmd(msg % (cmd, data, eol))
        return '%s: %s\n' % (cmd, tmp)

    def pressure(self):
        """ Retrieves current working pressure from pump. """
        ret = int(self.status(9))
        return ret

    def state(self):
        """{0:shutdown,1:Equil,2:ready, 3-11:steps 1-9, ... }"""
        ret = self.status(1)
        return int(ret)

    def status(self, status_type=None):
        """
        Pump Status
            0:?
            1:state
            2:total time
            3:step time
            4:flow
            5:%A
            6:%B
            7:%C
            8:%D
            9:pressure
            10:?
        """
        s = self.cmd('e').rstrip().split(',')
        if status_type is None:
            return s
        return s[status_type]

    def reset(self, seize=False):
        self.ser.write('\n\n\n')
        self.ser.flushInput()
        try:
            self.cmd('r')
            if seize:
                self.cmd('P')
            self.cmd('H')
        except (BadCmd, BadParam):
            msg = "We're having serious errors with the pump please"
            msg += " reset it and try again."
            print(msg)
            raise

    def _flush(self):
        print self.ser.read(self.ser.inWaiting()),

    def start(self):
        """ Starts executing current method """
        if self.state() is 0:
            self.cmd('S')

    def inject(self):
        """ Advances from step 0 to 1 (state: 2/3 to 4) """
        self.cmd('j')

    def stop(self):
        """ Stops Pumping immediately """
        self.cmd('s')

    def seize(self):
        """
            Seize external control
        """
        self.cmd('P')

    def release(self):
        """
            Release external control
        """
        self.cmd('r')

    def next(self):
        self.advance()

    def advance(self):
        """ advances sequentially from step 1 to 19 (state 4-22) """
        self.cmd('K')

    def resume(self):
        """ Resume from HLD
            Continue current method from pause. Also has the byproduct of
            transitioning from SHTDN to EQUIL without starting the pump.
        """
        self.cmd('J')

    def restart(self):
        """ Halt the method
            Safe Method restart.
        """
        self.cmd('H')

    def quit(self):
        """ Quit a method
            This only works from run states (i.e. > 0).
            Resets current method to step 0; time 0
        """
        self.cmd('Q')

    def info(self):
        """ Get general info about the pump: version.
            @return a,9991,200,9992,LC200 Pump: Version 1.08
        """
        self.cmd('I')

    def hold(self):
        """ Pauses Run HLD01
            Pauses *time* and gradient for current method.  Does NOT stop flow.
            Fails when in: SHTDN, EQUIL, READY
        """
        self.cmd('k')

    def _step(self, t=1, flow=0, A=0, B=0, C=0, D=0, curve=0):
        if sum((A, B, C, D)) != 100:
            D = 100-sum((A, B, C))
        return 'A,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%d' % (
            t, flow, A, B, C, D, 10 * curve)


def scan_devs():
    devs = [i for i in find_port() if i is not None and LC200Q.test(i)]
    print devs
    dev = LC200Q.test(devs[0])
    if dev:
        return dev

if __name__ == '__main__':
    pmp = LC200Q(scan_devs())

"""Explicit split-bus UART adapter for v11 IDs 1..14; never auto-enable torque.

Use the existing FEETECH model profiles. Calling send_result sends actual motion
packets; importing this module and opening/reading buses do not send motion.
"""
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent


def bus_for(servo_id):
    if type(servo_id) is not int or not 1 <= servo_id <= 14:
        raise ValueError('Only v11 physical servo IDs 1..14')
    return 'A' if servo_id <= 10 else 'B'


def packet(servo_id, instruction, params=()):
    bus_for(servo_id)
    if type(instruction) is not int or not 0<=instruction<=255:
        raise ValueError('Byte instruction required')
    if len(params)>120 or any(type(x) is not int or not 0<=x<=255 for x in params):
        raise ValueError('Bounded byte parameters required')
    body = bytes((servo_id,len(params)+2,instruction,*params))
    return b'\xff\xff'+body+bytes(((~sum(body))&255,))


def parse_reply(raw, servo_id, size):
    if len(raw)!=size+6 or raw[:2]!=b'\xff\xff' or raw[2]!=servo_id or raw[3]!=size+2 or ((~sum(raw[2:-1]))&255)!=raw[-1]:
        raise ValueError('Invalid servo reply')
    if raw[4]:
        raise RuntimeError('Servo error flags: '+str(raw[4]))
    return raw[5:-1]


class ServoBus:
    def __init__(self, transport_factory=None):
        if transport_factory is None:
            from combined_uart_transport import PmodServoTransport
            transport_factory = PmodServoTransport
        self.buses = {}
        self.last_sent = None
        self.previous_ticks = None
        sys.path.insert(0,str(ROOT/'model'))
        from piano_model import ServoCalibration
        self.cal = ServoCalibration(json.loads((ROOT/'model/servo_config.json').read_text(encoding='utf-8')))
        self.calibration = self.cal.servos
        try:
            for bus in ('A','B'):
                self.buses[bus] = transport_factory(pmod=bus)
        except Exception:
            self.close(); raise

    def exchange(self, servo_id, instruction, params=(), response_size=None):
        raw = self.buses[bus_for(servo_id)].transact_split(2,3,packet(servo_id,instruction,params),
                         response_length=0 if response_size is None else response_size+6)
        return b'' if response_size is None else parse_reply(raw,servo_id,response_size)

    def read_position(self, servo_id):
        raw = self.exchange(servo_id,2,(56,2),2)
        return int.from_bytes(raw,'big' if servo_id<=10 else 'little')

    def reset_from_feedback(self):
        self.previous_ticks = [self.read_position(i) for i in range(1,15)]
        self.last_sent = time.monotonic()
        return self.previous_ticks[:]

    def send_result(self, result):
        if result.get('command_valid') is not True:
            raise ValueError('v11 rejected the command')
        if self.previous_ticks is None:
            raise RuntimeError('Read encoders with reset_from_feedback before sending targets')
        commands = result['servo_commands_steps']
        if len(commands)!=14 or [x['id'] for x in commands]!=list(range(1,15)):
            raise ValueError('Complete ordered v11 command required')
        now = time.monotonic()
        elapsed = now-self.last_sent
        if not 0 < elapsed <= .1:
            raise RuntimeError('Command gap: reacquire feedback and reset controller')
        if result['joint_target_radians'][12] != 3.14:
            raise ValueError('Servo13 must stay at 3.14 rad')
        expected_ticks = self.cal.to_ticks(result['joint_target_radians'])
        if [item['position'] for item in commands] != expected_ticks:
            raise ValueError('Joint and encoder targets disagree')
        # Validate the complete frame before writing any channel.
        for i,(command,cal) in enumerate(zip(commands,self.calibration)):
            value = command['position']
            if type(value) is not int or not cal['minimum']<=value<=cal['maximum']:
                raise ValueError('Target outside calibration')
            speed = cal.get('max_speed_steps_s')
            if speed and abs(value-self.previous_ticks[i]) > speed*elapsed+1.0:
                raise ValueError('Tick slew exceeds calibrated speed (one-tick rounding allowance)')
        try:
            for command,cal in zip(commands,self.calibration):
                servo_id, value = command['id'], command['position']
                raw = value.to_bytes(2,'big' if servo_id<=10 else 'little')
                if cal.get('max_speed_steps_s'):
                    # STS registers 42..47: position, time=0, velocity cap.
                    # Vendor: https://docs.waveshare.com/ST3215_Servo/Serial-Debugging-Tool-Use
                    raw += bytes((0,0)) + int(cal['max_speed_steps_s']).to_bytes(2,'little')
                self.exchange(servo_id,3,(42,*raw))
        except Exception:
            self.previous_ticks = None
            raise
        self.previous_ticks = [x['position'] for x in commands]
        self.last_sent = now
        return dict(hardware_commands_sent=True,servo_ids=list(range(1,15)))

    def close(self):
        for bus in self.buses.values():
            bus.close()
        self.buses = {}

    def __enter__(self):return self
    def __exit__(self,*unused):self.close()

"""Shared two-PMOD output path. This deployment uses real UART internal loopback.

Model decoder steps are tagged as such; WM physical calibration is not certified.
External TX remains high for this user-authorized board-only test round.
"""
import json
import math
from pathlib import Path
from servo_bus import packet,bus_for

ROOT=Path(__file__).resolve().parent


def model_commands(model,result,waypoint_index=0):
    if model=='ppo':
        if result.get('command_valid') is not True:raise ValueError('PPO rejected command')
        commands=result['servo_commands_steps']
        if [c['id'] for c in commands]!=list(range(1,15)):raise ValueError('PPO ID order mismatch')
        return commands,0,'ppo_encoder_targets'
    if model!='wm' or result.get('accepted') is not True:raise ValueError('WM route not accepted')
    if type(waypoint_index) is not int or not 0<=waypoint_index<len(result['waypoints']):raise ValueError('Invalid WM waypoint')
    row=result['waypoints'][waypoint_index]
    ids=json.loads((ROOT/'wm_model/radian_mapping.json').read_text())['servo_ids']
    if sorted(ids)!=list(range(1,15)) or len(row['target_steps'])!=14:raise ValueError('WM ID layout mismatch')
    commands=sorted([dict(id=i,position=step) for i,step in zip(ids,row['target_steps'])],key=lambda x:x['id'])
    return commands,row['duration_ms'],'wm_model_decoder_steps_not_physical_calibration'


def output_frames(model,result,waypoint_index=0):
    commands,duration,encoding=model_commands(model,result,waypoint_index)
    frames=[]
    if type(duration) is not int or not 0<=duration<=65535:raise ValueError('Invalid waypoint duration')
    for c in commands:
        i,value=c['id'],c['position']
        if type(value) is not int or not 0<=value<=(1023 if i<=10 else 4095):raise ValueError('Encoder target width mismatch')
        endian='big' if i<=10 else 'little'
        raw=value.to_bytes(2,endian)
        if model=='wm' or i in (12,14):raw+=duration.to_bytes(2,endian)+(500 if i in (12,14) else 0).to_bytes(2,endian)
        frame=packet(i,3,(42,*raw))
        frames.append(dict(id=i,pmod=bus_for(i),tx_gpio=2,rx_gpio=3,position=value,
                           encoding=encoding,data=frame))
    return frames


class ModelUartOutput:
    def __init__(self,mode='internal_loopback',transport_factory=None):
        if mode!='internal_loopback':
            raise RuntimeError('This round permits internal UART only; physical WM calibration and feedback commissioning are outstanding')
        if transport_factory is None:
            from combined_uart_transport import PmodServoTransport
            transport_factory=PmodServoTransport
        self.buses={};self.mode=mode
        try:
            for name in ('A','B'):self.buses[name]=transport_factory(pmod=name)
        except Exception:self.close();raise

    def send_model_result(self,model,result,waypoint_index=0):
        # Validate every frame before starting either UART.
        frames=output_frames(model,result,waypoint_index)
        rows=[]
        for frame in frames:
            data=frame['data'];received=self.buses[frame['pmod']].diagnostic_loopback(data)
            if received!=data:raise RuntimeError('Model UART frame loopback mismatch')
            rows.append(dict(id=frame['id'],pmod=frame['pmod'],packet_hex=data.hex(),bytes=len(data)))
        return dict(routed_to='dual_pmod_uart',mode=self.mode,model=model,
            encoder_encoding=frames[0]['encoding'],frame_count=len(frames),
            pmod_A_ids=[r['id'] for r in rows if r['pmod']=='A'],
            pmod_B_ids=[r['id'] for r in rows if r['pmod']=='B'],frames=rows,
            loopback_passed=True,external_uart_bytes_sent=0,hardware_commands_sent=False)

    def close(self):
        for bus in self.buses.values():bus.close()
        self.buses={}

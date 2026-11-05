"""Verified four-bit status publisher. No separate 'turn WM LED on' command."""
ADDRESS = 0x40060000
PPO, WM, DEPTH640, UART320 = 1, 2, 4, 8


def mask(vision, model=None):
    if vision not in ('320','640') or model not in (None,'ppo','wm'):
        raise ValueError('Unknown status')
    if vision == '640' and model is not None:
        raise ValueError('This 640 overlay contains no model kernel')
    return (UART320 if vision=='320' else DEPTH640) | {None:0,'ppo':PPO,'wm':WM}[model]


def attach():
    from pynq import PL, MMIO
    if PL.ip_dict.get('status_led_gpio',{}).get('phys_addr') != ADDRESS:
        raise RuntimeError('Status LED output absent from resident overlay')
    return MMIO(ADDRESS,4096)


def publish(vision, model=None, io=None):
    value = mask(vision,model)
    io = attach() if io is None else io
    io.write(0,value)
    if int(io.read(0)) & 15 != value:
        raise RuntimeError('LED GPIO readback mismatch')
    return value


def dark(io=None):
    (attach() if io is None else io).write(0,0)

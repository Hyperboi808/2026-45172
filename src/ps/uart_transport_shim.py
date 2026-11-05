"""Existing PMOD entry point; reject UART use while 640 depth mode is loaded."""
import hashlib
import importlib.util
from pathlib import Path
import sys

ROOT=Path('/usr/local/lib/pynq-robot-models-v11-20261102-01')
_name='pynq_two_mode_uart_transport'

def _require_320():
    from pynq import PL
    path=Path(str(PL.bitfile_name))
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!='5e1d52189cc3ac7bde87a6e6ec7b8532d7acd0c58e32e05f31ec85a55e18ee85':
        raise RuntimeError('UART is available only in 320 mode; 640 is depth only')

# Preserve a stable public exception class for callers catching UART timeouts.
# It is imported without touching FPGA registers or changing its configuration.
spec=importlib.util.spec_from_file_location(_name,ROOT/'combined_uart_transport.py')
module=importlib.util.module_from_spec(spec);sys.modules[_name]=module;spec.loader.exec_module(module)
UartReceiveTimeout=module.UartReceiveTimeout

class PmodServoTransport(module.PmodServoTransport):
    def __init__(self,*args,**kwargs):
        _require_320();super().__init__(*args,**kwargs)
class PmodATransport(module.PmodATransport):
    def __init__(self,*args,**kwargs):
        _require_320();super().__init__(*args,**kwargs)
class PmodBTransport(module.PmodBTransport):
    def __init__(self,*args,**kwargs):
        _require_320();super().__init__(*args,**kwargs)

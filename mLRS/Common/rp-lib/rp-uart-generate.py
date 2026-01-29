import sys
import os

def generate_uart(name, serial_no, baud, tx_pin=None, rx_pin=None, invert_tx=False, invert_rx=False):
    with open('rp-uart-template.h', 'r') as f:
        content = f.read()

    content = content.replace('UART$', name.upper())
    content = content.replace('uart$', name.lower())
    
    header = f'#define {name.upper()}_USE_{serial_no.upper()}\n'
    header += f'#define {name.upper()}_BAUD {baud}\n'
    if tx_pin is not None:
        header += f'#define {name.upper()}_TX_PIN {tx_pin}\n'
    if rx_pin is not None:
        header += f'#define {name.upper()}_RX_PIN {rx_pin}\n'
    if invert_tx:
        header += f'#define {name.upper()}_INVERT_TX\n'
    if invert_rx:
        header += f'#define {name.upper()}_INVERT_RX\n'
    
    with open(f'rp-{name.lower()}.h', 'w') as f:
        f.write(header + content)

# Generate default UARTs for mLRS
# uart: generic/main
# uartb: second serial
# uartf: debug
generate_uart('uart', 'Serial', 115200)
generate_uart('uartb', 'Serial1', 115200)
generate_uart('uartf', 'Serial', 115200)

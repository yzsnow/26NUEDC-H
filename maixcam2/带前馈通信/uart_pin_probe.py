"""Read-only MaixCAM2 probe for the chassis telemetry RX pin.

Run this on the device before wiring.  It does not change any pin function and
does not start the motor.  Connect the chassis PA8/UART1_TX to one printed free
candidate, and connect both controller grounds.
"""

from maix import uart

try:
    from maix.peripheral import pinmap
except ImportError:
    from maix import pinmap


PIN = "A31"
PIN_LABEL = "PA31"
FUNCTION = "UART1_RX"

print("UART devices:", uart.list_devices())
if PIN not in pinmap.get_pins():
    print("{} ({}) is not exposed by this firmware".format(PIN_LABEL, PIN))
else:
    functions = pinmap.get_pin_functions(PIN)
    current = pinmap.get_pin_function(PIN)
    print("Configured RX: {}:{} (pinmap {}, current {})".format(
        PIN_LABEL, FUNCTION, PIN, current))
    print("Supported functions:", functions)
    print("Compatible:", FUNCTION in functions)

"""MaixCam UART1 setup for a 3.3 V MCU serial link."""

from maix import err, pinmap, uart

UART_DEVICE = "/dev/ttyS1"
UART_BAUD_RATE = 115200


def open_uart1():
    """Map A19/A18 to UART1 and return an opened 115200-8-N-1 port."""
    err.check_raise(
        pinmap.set_pin_function("A19", "UART1_TX"),
        "Failed to configure A19 as UART1_TX",
    )
    err.check_raise(
        pinmap.set_pin_function("A18", "UART1_RX"),
        "Failed to configure A18 as UART1_RX",
    )
    return uart.UART(UART_DEVICE, UART_BAUD_RATE)


if __name__ == "__main__":
    serial = open_uart1()
    serial.write(b"MAIXCAM_UART1_READY\r\n")

    # Add the application protocol here. Non-blocking reads return available
    # bytes; keep messages short and give both ends the same frame format.
    received = serial.read()
    if received:
        print(received)

import serial
import time
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from plotter import Plotter

# Startup string.
setup_string = """
********************************************************************************
*							 HELI REMOTE CONTROL							   *
********************************************************************************

Enter "CTRL"+"C" to write command.
Available commands:
	*STOP 			SPACE	0x00
	*SETPOINT 		SP		0x01
	*SET_PID0 		P0		0x02
	*SET_PID1 		P1		0x03
	*SET_DMAT0 		D0		0x04
	*SET_DMAT1 		D1		0x05
	*SET_DMAT2 		D2		0x06
	*QUIT			Q
"""

# Commands to send.
cmd_codes = {
    " ": b'\x00',
    "SP": b'\x01',
    "P0": b'\x02',
    "P1": b'\x03',
    "D0": b'\x04',
    "D1": b'\x05',
    "D2": b'\x06'
}

rf_nano = serial.Serial(
    port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', baudrate=115200, timeout=.1)


def write(data):
    print(f"Buffer sent: {data.decode()}\n")
    while (len(data) < 32):
        data += b'\x00'  # fill with zeros
    rf_nano.write(data[:32])  # send only first 32 bytes


def read():
    return rf_nano.readline()


def send_command():
    cmd = input()
    buf = b''
    if cmd not in cmd_codes.keys():
        return
    elif cmd in ["Q", "q", "quit"]:
        quit()
    elif cmd == "SP":
        buf = cmd_codes[cmd]
        for i in list(range(4)):
            buf += struct.pack("f",
                               float(input(f"Enter setpoint value [{i}]: ")))
    write(cmd_codes[cmd])


def heli_plotter():
    pico_serial = serial.Serial(
        port='/dev/serial/by-id/usb-Raspberry_Pi_Pico_E66138935F63992A-if00', baudrate=115200, timeout=.1)
    fig, ax = plt.subplots()
    plotter = Plotter(pico_serial, ax)
    ani = animation.FuncAnimation(fig, plotter.update, plotter.serial_getter, interval=1,
                                  blit=True, cache_frame_data=False)
    # ax.set_xlabel("")
    # ax.set_ylabel("")
    fig.canvas.manager.set_window_title('Helicopter data serial')
    fig.tight_layout()
    plt.show()


def main():
    print(setup_string)

    while True:
        try:
            data = read()
            print(data)
        except KeyboardInterrupt:
            send_command()


if __name__ == "__main__":
    main()
    # heli_plotter()

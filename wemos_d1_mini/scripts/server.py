#!/usr/bin/env python

from http.server import BaseHTTPRequestHandler, HTTPServer
from pprint import pprint
import time

setup_string = """
********************************************************************************
*							      REMOTE CONTROL							   *
********************************************************************************
Run "hostname -I" to get local ip address. Communication port is 8081.

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
    " ": '\x00',
    "SP": '\x01',
    "P0": '\x02',
    "P1": '\x03',
    "D0": '\x04',
    "D1": '\x05',
    "D2": '\x06'
}

new_cmd_flag = False  # flag to check new user commands
cmd_code = ''
cmd_data = ''

# Setup a simple server to read sensor data and send commands.
# Helicopter as client sends POST to share data and GET to obtain commands.


class ControllerServer(BaseHTTPRequestHandler):

    def do_GET(self):  # reply with last command
        global cmd_code, cmd_data, new_cmd_flag
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        message = f'{cmd_code},{cmd_data}'
        self.wfile.write(bytes(message, "utf8"))
        new_cmd_flag = False # command has been sent to client

    def do_POST(self):
        global new_cmd_flag
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        pprint('POST-request received. Data: ', post_data)
        # Send code 201 if a new command is available.
        self.send_response(200 if not new_cmd_flag else 201)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write("")


def get_command():
    global new_cmd_flag, cmd_code, cmd_data
    cmd = input()
    if cmd not in cmd_codes.keys():
        return
    elif cmd in ["Q", "q", "quit"]:
        quit()
    else:
        new_cmd_flag = True
        cmd_code = cmd_codes[cmd]
        cmd_data = input("Enter command data: ")


def main():
    while True:
        input("Enter command:")


def main():
    print(setup_string)
    server_address = ('127.0.0.1', 8081) # run "$ hostname -I" to get local address
    httpd = HTTPServer(server_address, ControllerServer)
    httpd.serve_forever()
    print(setup_string)
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            get_command()


if __name__ == "__main__":
    main()

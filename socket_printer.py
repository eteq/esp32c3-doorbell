from pathlib import Path
from time import time
import json
import socketserver


class PrintHandler(socketserver.StreamRequestHandler):
    def handle(self):
        print('hit handler')
        recv = self.rfile.readline()
        while recv.strip() == b'':
            recv = self.rfile.readline()
        print('got data at', time(), 'from', self.client_address, ':', recv)


if __name__ == "__main__":
    HOST, PORT = "", 65434

    with socketserver.TCPServer((HOST, PORT), PrintHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print('interrupted, closing server')
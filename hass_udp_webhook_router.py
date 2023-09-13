import os
from warnings import warn
from time import time
import socketserver

import requests

DEAD_TIME = 5  # seconds
ENDPOINT = os.environ.get('DOORBELL_WEBHOOK_ENDPOINT', 'default_endpoint')

if ENDPOINT == 'default_endpoint':
    warn('using default endpoint... this is probably not what we want')


endpoint_url = f'http://localhost:8123/api/webhook/{ENDPOINT}?dead_time={DEAD_TIME}'
def send_webhook():
    requests.put(endpoint_url)


last_send = [0]
class UDPReceiver(socketserver.BaseRequestHandler):
    def handle(self):
        data = self.request[0].strip()
        socket = self.request[1]
        print("{} wrote:".format(self.client_address[0]))
        print(data.decode())

        if data.startswith(b'triggered'):
            t = time()
            if (t - last_send[0]) > DEAD_TIME:
                print("Sending Webhook")
                send_webhook()
                last_send[0] = t


if __name__ == "__main__":
    HOST, PORT = "", 65434

    with socketserver.UDPServer((HOST, PORT), UDPReceiver) as server:
        server.serve_forever()

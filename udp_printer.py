import json
import socketserver

DATA_FN = 'udp_data.json'

fall_times = []
long_data = []
short_data = []
triggered = []
def parse_data(data):
    if data.strip() != b'':
        trigger, long, short = data.split(b'\n')
    fall_times.append(int(trigger.split(b':')[1]))
    long_data.append(eval(long.split(b':')[1]))
    short_data.append(eval(short.split(b':')[1]))
    triggered.append(b'NOT' not in trigger)

def save_data():
    # do the dumps and only write at the end so there's less time for a KeyboardInterrupt to corrupt the file
    s = json.dumps({'fall_times': fall_times, 'long_data': long_data, 'short_data': short_data, 'triggered': triggered})
    with open(DATA_FN, 'w') as f:
        f.write(s)
    

class UDPPrinterSaver(socketserver.BaseRequestHandler):
    def handle(self):
        data = self.request[0].strip()
        socket = self.request[1]
        print("{} wrote:".format(self.client_address[0]))
        print(data.decode())
        parse_data(data)
        save_data()

if __name__ == "__main__":
    HOST, PORT = "", 65434

    with socketserver.UDPServer((HOST, PORT), UDPPrinterSaver) as server:
        server.serve_forever()

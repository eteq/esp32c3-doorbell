from pathlib import Path
from time import time
import json
import socketserver

import numpy as np

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
plt.ioff()

NTOPLOT = 5

TICKS_PER_SECOND = 16000000
TICKS_PER_USECOND = TICKS_PER_SECOND/1000000

datafile = Path('last_data.json')

data = []
if datafile.is_file():
    print('Loading previous data from', datafile)
    with datafile.open() as f:
        data.extend(json.load(f))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 10))
ax3 = ax2.twinx()


def update_plot():
    ax1.cla()
    for d in data[-NTOPLOT:]:
        ax1.scatter(np.arange(len(d['t'])), np.array(d['t'])/TICKS_PER_USECOND)
        
    ax1.set_xlabel('n')
    ax1.set_ylabel('Touch sensor time [us]')

    alldata = np.concatenate([d['t'] for d in data])/TICKS_PER_USECOND
    alllastndata = np.concatenate([d['t'] for d in data[-NTOPLOT:]])/TICKS_PER_USECOND
    ax2.cla()
    _, bins, _ = ax2.hist(alldata, bins='auto')
    mean = np.mean(alldata)
    std = np.std(alldata)
    ax2.axvline(mean, color='C0', alpha=.8, linestyle='--')
    ax2.axvline(mean+std, color='C0', alpha=.8, linestyle=':')
    ax2.axvline(mean-std, color='C0', alpha=.8, linestyle=':')

    ax2.set_xlabel('Touch sensor time [us]')
    ax2.set_ylabel('n', color='C0')
    ax2.tick_params(axis='y', labelcolor='C0')

    ax3.cla()
    ax3.hist(alllastndata, histtype='step', bins=bins, color='C1')
    #ax3.set_ylabel(f'n ( {NTOPLOT} latest)', color='C1')
    ax3.tick_params(axis='y', labelcolor='C1')

    mean = np.mean(alllastndata)
    std = np.std(alllastndata)
    ax3.axvline(mean, color='C1', alpha=.8, linestyle='--')
    ax3.axvline(mean+std, color='C1', alpha=.8, linestyle=':')
    ax3.axvline(mean-std, color='C1', alpha=.8, linestyle=':')

    fig.tight_layout()
    plt.draw()
    plt.savefig('plot.png')

class PlotHandler(socketserver.StreamRequestHandler):
    def handle(self):
        recv = self.rfile.readline().strip()
        if recv == '':
            return
        print('got data at', time(), 'from', self.client_address)
        
        data.append(json.loads(recv.decode("utf-8")))
        update_plot()


if __name__ == "__main__":
    if data:
        update_plot()

    HOST, PORT = "", 65432

    with socketserver.TCPServer((HOST, PORT), PlotHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print('interrupted, saving data and closing server')
            with datafile.open('w') as f:
                json.dump(data, f)
from flask import Flask, send_file  

app = Flask(__name__)

@app.route("/")
def hello_world():
    return '<img src="plot.png">'

@app.route("/plot.png")
def plot():
    return send_file('plot.png', mimetype='image/png', max_age=0)
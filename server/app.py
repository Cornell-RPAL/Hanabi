from flask import Flask, jsonify, request, json
import sys
sys.path.insert(0, '../game/')
from alexaGame import Hanabi


app = Flask(__name__)
h = Hanabi()

@app.route('/launch', methods=['GET'])
def launch():
    
    return "Welcome to Hanabi."

def colorHint(color):
    h.update("hint " + color)
    return h.output

def numberHint(number):
    return 'Number hint received!'

def hintRequest():
    return 'Hint'

@app.route('/hintColor', methods=['POST'])
def hintColorResult():
color = request.get_json()['color']
quant = request.get_json()['quantity']
return colorHint(color, quant)

@app.route('/hintNumber', methods=['POST'])
def hintNumberResult():
    number = request.get_json()['number']
    quant = request.get_json()['quantity']
    return numberHint(number, quant)

@app.route('/hintRequest', methods=['GET'])
def hintRequestResult():
    return hintRequest()

if __name__ == '__main__':
   app.run(host='0.0.0.0', port=5000, debug=True)
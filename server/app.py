from flask import Flask, jsonify, request, json

app = Flask(__name__)

# class JsonResponce():
#     def __init__(self, message):
#         self._message = message

@app.route('/')
def index():
    return jsonify("Hello, World!")

def colorHint(color,quantity):
    return 'Color hint received!'

def numberHint(number,quantity):
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
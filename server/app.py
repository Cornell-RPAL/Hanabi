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

def numberHint(color,quantity):
    return 'Number hint received!'

def hintRequst():
    return 'Hint'

@app.route('/alexa', methods=['POST'])
def result():
   # resp = ''
   intent = request.get_json()['intent']
   if intent == 'HintColorIntent':
       color = request.get_json()['color']
       quant = request.get_json()['quantity']
       resp = colorHint(color, quant)
   elif intent == 'HintNumberIntent':
       number = request.get_json()['number']
       quant = request.get_json()['quantity']
       resp = numberHint(number, quant)
   elif intent == 'HintRequestIntent':
       resp = hintRequst()
   else:
       resp = 'Invalid Input'
   return resp


if __name__ == '__main__':
   app.run(host='0.0.0.0', port=5000, debug=True)
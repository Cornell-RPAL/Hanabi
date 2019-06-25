export FLASK_APP=app.py # point the Flask app at app.py
export FLASK_ENV=development # tell Flask we want debugging
flask run --host=0.0.0.0 # make the app accessible externally
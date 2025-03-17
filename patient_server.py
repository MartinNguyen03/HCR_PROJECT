
from flask import Flask, send_from_directory

app = Flask(__name__, static_folder='Static')

@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'start.html')

@app.route('/<path:filename>')
def serve_static(filename):
   
    return send_from_directory(app.static_folder, filename)
@app.route('/display_rfid_temperature')
def display_rfid_temperature():
    return send_from_directory(app.static_folder, 'display_rfid_temperature.html')

if __name__ == '__main__':
    app.run(debug=True)
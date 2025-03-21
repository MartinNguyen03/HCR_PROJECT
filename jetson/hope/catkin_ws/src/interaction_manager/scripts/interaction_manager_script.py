#!/usr/bin/env python3


import os
import rospy
import signal
import sys
import depthai as dai
from flask import Flask, render_template, request, redirect, url_for, jsonify, Response
import requests
from std_msgs.msg import String, Float32
from hope_msgs.msg import blood_pressure_msg
import json
import cv2
import depthai as dai
import threading
# Explicitly specify the path to the templates folder
app = Flask(__name__, template_folder='templates', static_url_path='/static')


rospy.init_node('interaction_manager', anonymous=True)

# ROS publishers
global motion_pub
global nav_pub
motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=1)
state_pub = rospy.Publisher('/interaction_manager/current_interaction_state', String, queue_size=1,latch=False)
bp_pub = rospy.Publisher('/sensor_manager/blood_pressure', blood_pressure_msg, queue_size=10)
body_temp_pub = rospy.Publisher('/sensor_manager/body_temperature', Float32, queue_size=10)
nav_pub = rospy.Publisher('/nav_module/status', String, queue_size=1)

# State machine transitions
states = [
    {"name": "SANITATION_STATUS", "next": "IDLE", "skip": None},
    {"name": "QUEST", "next": None, "skip": None},
    {"name": "MANUAL_CONTROL", "next": None, "skip": None},
    {"name": "START_INTERACTION", "next": "MEASURE_BODY_TEMPERATURE", "skip": "THANK_PATIENT"},
    {"name": "SCAN_PATIENT_ID", "next": "MEASURE_BODY_TEMPERATURE", "skip": "THANK_PATIENT"},
    {"name": "DISPLAY_PATIENT_INFO", "next": "MEASURE_BODY_TEMPERATURE", "skip": "THANK_PATIENT"},
    {"name": "MEASURE_BODY_TEMPERATURE", "next": None, "skip": None},
    {"name": "DISPLAY_BODY_TEMPERATURE", "next": "MEASURE_HEART_RATE_AND_SPO2_INSTRUCTION", "skip": "MEASURE_BLOOD_PRESSURE_INSTRUCTION"},
    {"name": "MEASURE_HEART_RATE_AND_SPO2_INSTRUCTION", "next": "MEASURE_HEART_RATE_AND_SPO2", "skip": "MEASURE_BLOOD_PRESSURE_INSTRUCTION"},
    {"name": "MEASURE_HEART_RATE_AND_SPO2", "next": None, "skip": None},
    {"name": "DISPLAY_HEART_RATE_AND_SPO2", "next": "MEASURE_BLOOD_PRESSURE_INSTRUCTION", "skip": "MEASURE_ECG_INSTRUCTION"},
    {"name": "MEASURE_BLOOD_PRESSURE_INSTRUCTION", "next": "MEASURE_BLOOD_PRESSURE", "skip": "MEASURE_ECG_INSTRUCTION"},
    {"name": "MEASURE_BLOOD_PRESSURE", "next": "MEASURE_ECG_INSTRUCTION", "skip": None},
    {"name": "MEASURE_ECG_INSTRUCTION", "next": "MEASURE_ECG", "skip": "UPLOAD_SENSOR_DATA"},
    {"name": "MEASURE_ECG", "next": None, "skip": None},
    {"name": "UPLOAD_SENSOR_DATA", "next": None, "skip": None},
    {"name": "ASK_PRESET_QUESTIONS_SETUP", "next": "ASK_PRESET_QUESTIONS", "skip": "THANK_PATIENT"},
    {"name": "ASK_PRESET_QUESTIONS", "next": None, "skip": None},
    {"name": "THANK_PATIENT", "next": None, "skip": None},
    {"name": "IDLE", "next": None, "skip": None}
]

# Track the current state
current_state_index = 0

@app.route('/')
def home():
    """Redirects to the initial state."""
    return redirect(url_for('state_page', state_name=states[current_state_index]["name"]))

@app.route('/<state_name>')
def state_page(state_name):
    """Renders the page for the given state, updates index, and extracts query parameters."""
    global current_state_index
    for i, state in enumerate(states):
        if state["name"] == state_name:
            current_state_index = i
            publish_state()

            # Extract query parameters (e.g., temp, heart_rate, etc.)
            query_params = request.args.to_dict()
            
            if state_name == "MEASURE_BODY_TEMPERATURE":
                rospy.sleep(6)
                def fetch_temp_background():
                    for _ in range(3):
                        status = fetch_and_publish_temp()
                        if status == True:
                            rospy.loginfo("Successfully fetched temperature from ESP")
                            return 0
                        rospy.sleep(5)
                    rospy.loginfo("Failed to fetch temperature from ESP. Sending 0.")
                    body_temp_pub.publish(0)

                # Run fetch_and_publish_temp() in a separate thread
                threading.Thread(target=fetch_temp_background, daemon=True).start()
                    
            return render_template(f'{state_name}.html', state=state, **query_params)
    return "State not found", 404

def fetch_and_publish_temp():
    try:
        response = requests.post(f"http://192.168.1.133/get_temp", data={"request_temp": "true"}, timeout=5)
        response_json = response.json()

        if "generated_temp" in response_json:
            temperature = float(response_json["generated_temp"])
            if temperature < 50 and temperature > 15:
                rospy.loginfo(f"Received temperature from ESP: {temperature}")
                body_temp_pub.publish(temperature)
                rospy.loginfo(f"Published body temperature: {temperature}")
                return True
            else:
                rospy.loginfo("Temperature out of range")
                return False
        else:
            rospy.loginfo("Temperature not received from ESP")
            return False

    except requests.exceptions.RequestException as e:
        rospy.logerr(f"Could not connect to the ESP: {str(e)}")
        return False


        

def publish_state():
    """Publishes the current state name to ROS."""
    state_pub.publish(states[current_state_index]["name"])
    print(states[current_state_index]["name"])

@app.route('/next')
def next_state():
    """Moves to the next state if available."""
    global current_state_index
    if states[current_state_index]["next"]:
        target_state = states[current_state_index]["next"]
        current_state_index = next(i for i, state in enumerate(states) if state["name"] == target_state)
    return redirect(url_for('state_page', state_name=states[current_state_index]["name"]))

@app.route('/skip')
def skip_state():
    """Moves to the skip state if defined."""
    global current_state_index
    if states[current_state_index]["skip"]:
        target_state = states[current_state_index]["skip"]
        current_state_index = next(i for i, state in enumerate(states) if state["name"] == target_state)
    return redirect(url_for('state_page', state_name=states[current_state_index]["name"]))

''' 
SOME CAMERA SHIT
'''
def generate():
    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("video_stream")
    camRgb.video.link(xoutRgb.input)

    with dai.Device(pipeline) as device:
        video_queue = device.getOutputQueue(name="video_stream", maxSize=4, blocking=False)
        
        while True:
            try:
                frame = video_queue.get().getCvFrame()
                if frame is None:
                    continue

                _, encodedImage = cv2.imencode(".jpg", frame)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' +
                       bytearray(encodedImage) + b'\r\n')
            except Exception as e:
                rospy.logerr(f"Video stream error: {str(e)}")
                break  # Stop loop on error

@app.route("/video_feed")
def video_feed():
    return Response(generate(), mimetype = "multipart/x-mixed-replace; boundary=frame")
      
@app.route('/submit_bp', methods=['POST'])
def submit_bp():
    print("in submit_bp")

    try:
        # Parse JSON data
        data = request.json  
        systolic = int(data.get('systolic', 0))
        diastolic = int(data.get('diastolic', 0))

        # Validate data
        if systolic <= 0 or diastolic <= 0:
            return "Invalid blood pressure values", 400

        # Create and publish ROS message
        bp_msg = blood_pressure_msg()
        bp_msg.systolic = systolic
        bp_msg.diastolic = diastolic
        bp_pub.publish(bp_msg)
        rospy.loginfo(f"Blood pressure: {systolic}/{diastolic}")

        return "200 OK", 200

    except Exception as e:
        rospy.logerr(f"Error in /submit_bp: {str(e)}")
  
  
  
        return "500 Internal Server Error", 500


@app.route("/move", methods=["POST"])
def move():
    """Handles movement commands (WASD keys)."""
    global motion_pub
    global nav_pub
    data = request.json
    direction = data.get("direction")

    if direction in ["forward", "backward", "left", "right","stop"]:
        if direction == "stop":
            nav_pub.publish("Goal Reached")
            rospy.loginfo(f"Published nav command: Goal Reached")
            return jsonify({"status": "success", "command": direction}), 200
        else:
            motion_pub.publish(direction)
            rospy.loginfo(f"Published motion command: {direction}")
            return jsonify({"status": "success", "command": direction})
    else:
        return jsonify({"status": "error", "message": "Invalid command"}), 400

def signal_handler(sig, frame):
    """Handles keyboard interrupt and ensures a clean shutdown."""
    print("\nShutting down Flask server and ROS node gracefully...")
    rospy.signal_shutdown("Keyboard Interrupt")
    sys.exit(0)

if __name__ == '__main__':
    # Handle Ctrl+C to exit cleanly
    signal.signal(signal.SIGINT, signal_handler)


    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    except KeyboardInterrupt:
        signal_handler(None, None)

#$ rosrun interaction_manager interaction_manager_script.py
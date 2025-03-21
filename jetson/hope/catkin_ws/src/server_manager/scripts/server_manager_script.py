#!/usr/bin/env python3

import rospy
import requests

import time
from datetime import datetime

import json

from std_msgs.msg import String
from std_msgs.msg import Bool

from hope_msgs.msg import patient_info_msg
from hope_msgs.msg import sensor_data_msg 
from hope_msgs.msg import string_arr_msg

API_BASE_URL = "http://192.168.1.226:8000" 

global patient_to_visit_pub
global upload_state_pub
global data_packet_to_server

class DataPacket:
    def __init__(self, visit_id=0, sensor_data=None, patient_answers=None):
        self.visit_id = visit_id
        self.sensor_data = sensor_data if sensor_data is not None else {}
        self.patient_answers = patient_answers if patient_answers is not None else []

    def __repr__(self):
        return f"DataPacket(visit_id={self.visit_id}, sensor_data={self.sensor_data}, patient_answers={self.patient_answers})"

data_packet_to_server = DataPacket()

def get_all_scheduled_visits():
    """
    Fetches the list of scheduled visits from the server for all patients.
    """
    try:
        response = requests.get(f"{API_BASE_URL}/all-scheduled-visits/")
        if response.status_code == 200:
            visits = response.json()
            return visits if visits else None
        else:
            rospy.logwarn("Error fetching visits: %s", response.text)
            return None
    except Exception as e:
        rospy.logerr("Connection error: %s", e)
        return None




def handle_state_machine_status_callback(msg):
    if msg.data == "ReadPatientQueue":
        send_patient_to_visit()
        rospy.loginfo("Published patient to visit")
    elif msg.data == "AskPresetQuestions":
        send_patient_questions()
        rospy.loginfo("Published patient questions")

def send_patient_questions_fake():
    global patient_questions_pub
    questions = ["How are you feeling today?", "Are you experiencing any pain?", "Are you experiencing any swelling around your ankles?", "Are you experiencing any shortness of breath?"]
    question_msg = string_arr_msg()
    question_msg.data = questions
    patient_questions_pub.publish(question_msg)
    rospy.loginfo("Published patient questions: %s", questions)

def send_patient_questions():
    #visits = get_all_scheduled_visits() #or 
    visits = get_all_scheduled_visits_fake()
    visits_json = json.dumps(visits)
    global patient_questions_pub
    if visits:
        for visit in visits:
            try:
                scheduled_time = datetime.strptime(visit["scheduled_time"], "%Y-%m-%dT%H:%M:%S")
                now = datetime.now()
                if now >= scheduled_time:
                    question_msg = string_arr_msg()
                    question_msg.data = visit["doctor_questions"]
                    patient_questions_pub.publish(question_msg)
                    rospy.loginfo("Published patient questions: %s", question_msg)
                    break
            except Exception as e:
                    rospy.logerr("Error processing visit %s: %s", str(visit), e)
    

def get_coords_from_room(room):
    """
    Returns the x, y coordinates for a given room.
    """
    if room == "A":
        return (3.3, 4.4)
    elif room == "B":
        return (5.5, 6.6)
    elif room == "C":
        return (7.7, 8.8)
    else:
        return (0, 0)

def get_all_scheduled_visits_fake():
    """
    A mock function returning two visits in the past for easy testing.
    """
    return [{"id":21,"patient_id":3,"scheduled_time":"2025-03-16T21:30:00","status":"scheduled","batch_id":"","doctor_comments":"hii","doctor_questions":["How are you feeling today?","Are you experiencing any pain?","Are you happy in life","Are you experiencing any shortness of breath?","Rate this expereince"],"room":"B"},
            {"id":22,"patient_id":3,"scheduled_time":"2025-03-16T22:30:00","status":"scheduled","batch_id":"","doctor_comments":"","doctor_questions":["Are you experiencing any pain?","Are you experiencing any swelling around your ankles?","Are you experiencing any shortness of breath?","Did I do a good job?"],"room":"B"}
            ]

def send_patient_to_visit_fake():
    global patient_to_visit_pub
    patient_info = patient_info_msg()

    patient_info.visit_id = 21
    patient_info.patient_name = "John Doe"
    patient_info.patient_id = 3
    patient_info.patient_x_coord = 3.3
    patient_info.patient_y_coord = 4.4

    patient_to_visit_pub.publish(patient_info)
    rospy.loginfo("Published patient to visit: %s", patient_info)


def send_patient_to_visit():
    global patient_to_visit_pub
    #NOTE: This is currently fake data. Will have to change this.
    visits = get_all_scheduled_visits_fake()
    visits_json = json.dumps(visits)
    
    if visits:
        for visit in visits:
            try:
                scheduled_time = datetime.strptime(visit["scheduled_time"], "%Y-%m-%dT%H:%M:%S")
                now = datetime.now()
                if now >= scheduled_time:
                    patient_info = patient_info_msg()
                    patient_info.visit_id = visit["id"]
                    patient_info.patient_id = visit["patient_id"]
                    coords = get_coords_from_room(visit["room"])
                    patient_info.patient_x_coord = coords[0]
                    patient_info.patient_y_coord = coords[1]
                    patient_to_visit_pub.publish(patient_info)
                    rospy.loginfo("Published patient to visit: %s", patient_info)
                    break
            except Exception as e:
                    rospy.logerr("Error processing visit %s: %s", str(visit), e)
    
    rospy.loginfo("Published all patients to visit")


# rostopic pub -1 /server_manager/upload_sensor_data hope_msgs/sensor_data_msg "{visit_id: 20, body_temperature: 36.5, heart_rate: 75, spo2: 98, systolic: 120, diastolic: 80, ecg_data: [0.1, 0.2, 0.3, 0.4]}"
def upload_sensor_data_callback(msg):
    """
    Callback for the 'visit_completed' topic.
    Sends sensor data back to the FastAPI server via complete_visit().
    """
    global data_packet_to_server

    data_packet_to_server.visit_id = msg.visit_id

    data_packet_to_server.sensor_data = {
        "temperature": msg.body_temperature,
        "heartrate": msg.heart_rate,
        "spo2": msg.spo2,
        "systolic": msg.systolic,
        "diastolic": msg.diastolic,
        "ecg": list(msg.ecg_data)  # Convert ROS float32[] to a Python list
    }

    rospy.loginfo("Received completion for visit id: %d", data_packet_to_server.visit_id)
    rospy.loginfo("Sensor data: %s", data_packet_to_server.sensor_data)
    #upload_sensor_data_to_server(visit_id, sensor_data)

# rostopic pub -1 /server_manager/question_details_answered hope_msgs/string_arr_msg "data: ['Good', 'No', 'No', 'New data']"
def upload_patient_responses_callback(msg):
    rospy.loginfo("Received patient responses: %s", msg.data)
    global data_packet_to_server
    data_packet_to_server.patient_answers = msg.data
    upload_data_to_server(data_packet_to_server.visit_id, data_packet_to_server.sensor_data, data_packet_to_server.patient_answers)

def upload_data_to_server(visit_id, sensor_data, patient_answers):
    """
    Sends completed visit data back to the server via POST /update-visit.
    """
    global upload_state_pub
    payload = {
        "visit_id": visit_id,
        "sensor_data": sensor_data,
        "patient_answers": patient_answers
    }
    try:
        resp = requests.post(f"{API_BASE_URL}/update-visit/", json=payload, timeout=5)
        if resp.status_code == 200:
            rospy.loginfo("Visit %d completed. Data sent! Response: %s", visit_id, resp.text)
            upload_state_pub.publish(True)
        else:
            rospy.logwarn("Error updating visit %d (status %d): %s",
                          visit_id, resp.status_code, resp.text)
            
    except requests.RequestException as e:
        rospy.logerr("Connection error while updating visit %d: %s", visit_id, e)
        #THis is temp for testing
        upload_state_pub.publish(True)




if __name__ == '__main__':
    rospy.init_node('server_manager')

    # Publisher for individual instructions
    global patient_to_visit_pub
    global upload_state_pub

    patient_to_visit_pub = rospy.Publisher('/server_manager/patient_to_visit', patient_info_msg, queue_size=10)
    upload_state_pub = rospy.Publisher('/server_manager/upload_status', Bool, queue_size=10)
    patient_questions_pub = rospy.Publisher('/server_manager/question_details', string_arr_msg, queue_size=10)
    # Publisher for the entire visits list as JSON
    # all_visits_pub = rospy.Publisher('all_scheduled_visits', String, queue_size=10)

    # Subscriber for visit completion
    rospy.Subscriber('/server_manager/upload_sensor_data', sensor_data_msg, upload_sensor_data_callback)
    rospy.Subscriber('/state_machine/status', String, handle_state_machine_status_callback)
    rospy.Subscriber('/server_manager/question_details_answered', string_arr_msg, upload_patient_responses_callback)
    rospy.loginfo("Server manager node started")
    # Start background thread for scheduled visits
    # thread = threading.Thread(target=check_scheduled_visits,
    #                           args=(visit_instruction_pub, all_visits_pub))
    # thread.daemon = True
    # thread.start()

    rospy.spin()
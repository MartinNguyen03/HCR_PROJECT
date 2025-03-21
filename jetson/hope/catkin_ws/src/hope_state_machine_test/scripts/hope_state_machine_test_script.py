#!/usr/bin/env python3

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool, Int32, Float32
from hope_msgs.msg import patient_info_msg
from hope_msgs.msg import ecg_data_msg
from hope_msgs.msg import string_arr_msg
from hope_msgs.msg import sensor_data_msg
from hope_msgs.msg import blood_pressure_msg
from hope_msgs.msg import heart_rate_and_spo2_msg

from geometry_msgs.msg import Point


global sensor_data_msg_state_machine
sensor_data_msg_state_machine = sensor_data_msg()

global jetson_ip
jetson_ip = "192.168.1.102:5000" # Change this to the Jetson's IP address

# # Utility function for retries
# def retry_logic(max_retries, check_function, retry_message):
#     for attempt in range(max_retries):
#         if check_function():
#             return True
#         rospy.loginfo(retry_message)
#         
#     return False

# -------------------------
# Pre-Patient States
# -------------------------

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['read_patient_queue'])

        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
    

    def execute(self, userdata):
        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.screen_pub.publish(f"http://{jetson_ip}/IDLE")

        rospy.loginfo("resetting robot")
        self.motion_pub.publish("AL_off")
        rospy.sleep(10)
        rospy.loginfo("waking robot up")
        self.motion_pub.publish("wake_up")
        rospy.sleep(10)
        self.motion_pub.publish("face_track_off")

        rospy.loginfo("Robot is idle, waiting for a task...")
        
        global sensor_data_msg_state_machine
        sensor_data_msg_state_machine.visit_id = 0

        rospy.sleep(5)
        return 'read_patient_queue'


class ReadPatientQueue(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sanitised', 'alert_nurse'], output_keys=['patient_x', 'patient_y'])

        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.queue_sub = rospy.Subscriber('/server_manager/patient_to_visit', patient_info_msg, self.queue_callback)

        self.sanitised = False  
        self.patient_pending = False
        self.patient_x_coord = 0
        self.patient_y_coord = 0

# test message: rostopic pub -1 /server_manager/visit_schedule hope_msgs/patient_info_msg 10 "Adi" 1 1 1
    def queue_callback(self, msg):
        if msg.patient_id == 0:
            rospy.loginfo("No patients in the queue. Will check again in 10 seconds")
            self.queue_pub.publish("Requesting patient queue")
        else:
            rospy.loginfo("Recieved patient queue data.")
            rospy.loginfo(f"Visit ID: {msg.visit_id}")
            rospy.loginfo(f"Patient Name: {msg.patient_name}")
            rospy.loginfo(f"Patient ID: {msg.patient_id}")
            rospy.loginfo(f"Patient X coordinates: {msg.patient_x_coord}")
            rospy.loginfo(f"Patient Y coordinates: {msg.patient_y_coord}")
            
            global sensor_data_msg_state_machine
            sensor_data_msg_state_machine.visit_id = msg.visit_id
            sensor_data_msg_state_machine.body_temperature = 0
            sensor_data_msg_state_machine.heart_rate = 0
            sensor_data_msg_state_machine.spo2 = 0
            sensor_data_msg_state_machine.systolic = 0
            sensor_data_msg_state_machine.diastolic = 0
            sensor_data_msg_state_machine.ecg_data = [0] * 50


            self.patient_x_coord = msg.patient_x_coord
            self.patient_y_coord = msg.patient_x_coord

            self.patient_pending = True


    def execute(self, userdata):
        self.patient_pending = False
        self.sanitised = False 
        
        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)

        while not rospy.is_shutdown() and not self.patient_pending:
            rospy.loginfo("Waiting for patient queue...")
            rospy.sleep(1)

        rospy.loginfo("Patient found in the queue.")
        userdata.patient_x = self.patient_x_coord
        userdata.patient_y = self.patient_y_coord

        if self.sanitised:
            rospy.loginfo("Sanitation confirmed.")
            return 'sanitised'
        else:
            rospy.loginfo("Sanitation required.")
            return 'alert_nurse'


class AlertNurseForSanitation(smach.State):
    def __init__(self):
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        smach.State.__init__(self, outcomes=['sanitised'])
        self.nurse_pub = rospy.Publisher('/server_manager/alert_nurse', Bool, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.sanitised = False
    
    def interaction_callback(self, msg):
        if msg.data == "IDLE":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.sanitised = True
       

    def execute(self, userdata):
        self.sanitised = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        
        rospy.loginfo("Alerting nurse for sanitation.")
        self.nurse_pub.publish(True)

        global jetson_ip
        self.screen_pub.publish(f"http://{jetson_ip}/SANITATION_STATUS")


        while not rospy.is_shutdown() and not self.sanitised:
            rospy.loginfo("Waiting for nurse to complete sanitation...")
            self.audio_pub.publish("Waiting for nurse to complete sanitation...")
            rospy.sleep(10)

        self.audio_pub.publish("Sanitation completed.")

        rospy.loginfo("Sanitation completed.")

        return 'sanitised'


class TravelToPatient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_interaction'], input_keys=['patient_x', 'patient_y'])

        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
        self.nav_goal_pub = rospy.Publisher('/nav_module/goal', Point, queue_size=10)
        self.nav_status_sub = rospy.Subscriber('/nav_module/status', String, self.nav_status_callback)
        self.goal_reached = False

#rostopic pub -1 /nav_module/status std_msgs/String "Goal Reached"
    def nav_status_callback(self, msg):
        if msg.data == "Goal Reached":
            self.goal_reached = True

    def execute(self, userdata):
        self.goal_reached = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")
        self.motion_pub.publish("face_straight")
        self.screen_pub.publish(f"http://{jetson_ip}/IDLE")


        rospy.loginfo("Travelling to the patient. Can also do manual control")

        nav_goal = Point()
        nav_goal.x = userdata.patient_x
        nav_goal.y = userdata.patient_y
        self.nav_goal_pub.publish(nav_goal)
        rospy.sleep(2)

        while not rospy.is_shutdown() and not self.goal_reached:
            rospy.loginfo("Waiting to reach the patient...")
            rospy.sleep(5)

        rospy.loginfo("Reached the patient.")
        return 'start_interaction'


# -------------------------
# Interaction States
# -------------------------

class StartInteraction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'exit_interaction'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        rospy.loginfo("StartInteraction state initialized")

        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.interaction_complete = False

    def interaction_callback(self, msg):
        
        rospy.loginfo(f"DEBUG: Received message -> {msg.data}")
        self.next_state = msg.data
        if msg.data == "MEASURE_BODY_TEMPERATURE" or msg.data == "UPLOAD_SENSOR_DATA":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True
     
    def execute(self, userdata):
        self.interaction_complete = False
        
        self.motion_pub.publish("wake_up")

        rospy.loginfo("Sensor data reset.")

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("face_track_on")
        global jetson_ip
        self.screen_pub.publish(f"http://{jetson_ip}/START_INTERACTION")
        #self.motion_pub.publish("wave")
        self.audio_pub.publish("Hello, I am HOPE, your medical assistant robot. If you're ready to begin, please press the start button on the screen. If you do not consent or not available at this moment to have your vitals taken, please press the exit checkup button.")
        rospy.sleep(5)

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)
        
        if self.next_state == "MEASURE_BODY_TEMPERATURE":
            return 'next'
        else:
            return 'exit_interaction'

#Not using this right now
class ScanPatientID(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'exit_interaction'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.sensor_pub = rospy.Publisher('/sensor_manager/sensor_command', String, queue_size=10)
        self.interaction_pub = rospy.Publisher('/interaction_manager/patient_epc_code', String, queue_size=10)

        self.sensor_patient_epc_code_sub = rospy.Subscriber('/sensor_manager/patient_epc_code', String, self.sensor_patient_epc_code_callback)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False

    def sensor_patient_epc_code_callback(self, msg):
        self.interaction_pub.publish(msg.data) # Forwarding the patient EPC code to the interaction manager
        rospy.loginfo(f"Patient EPC code: {msg.data}")
    
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "DISPLAY_PATIENT_INFO" or msg.data == "THANK_PATIENT":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True
    
    def execute(self, userdata):
        self.interaction_complete = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)

        rospy.loginfo("Scanning patient ID...")
        self.motion_pub.publish("raise_right_arm")
        self.audio_pub.publish("Bring your wrist closer to the scanner.")
        self.sensor_pub.publish("patient_epc_code")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)

        if self.next_state == "DISPLAY_PATIENT_INFO":
            return 'next'
        else:
            return 'exit_interaction'

#Not using this right now
class DisplayPatientInfo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'exit_interaction'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
    
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "MEASURE_BODY_TEMPERATURE" or msg.data == "THANK_PATIENT":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True
        
    def execute(self, userdata):
        self.interaction_complete = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)

        self.audio_pub.publish("Please check that your details are correct. If you are happy and wish to move to the measurement of your body temperature, please press the start button. If you are not happy, please press the exit checkup button.")
        self.motion_pub.publish("lower_right_arm")

        while not rospy.is_shutdown() and not interaction_complete:
            rospy.loginfo("Waiting for the user to accept patient details")
            rospy.sleep(5)
        
        if self.next_state == 'MEASURE_BODY_TEMPERATURE':
            return 'next'
        else:
            return 'exit_interaction'


class MeasureBodyTemperature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__
    
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.sensor_pub = rospy.Publisher('/sensor_manager/sensor_command', String, queue_size=10)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)

        self.sensor_patient_epc_code_sub = rospy.Subscriber('/sensor_manager/body_temperature', Float32, self.sensor_body_temperature_callback)
        self.received_data = False
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
        
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "DISPLAY_BODY_TEMPERATURE" or msg.data == "MEASURE_HEART_RATE_AND_SPO2_INSTRUCTION":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True

#rostopic pub -1 /sensor_manager/body_temperature std_msgs/Float32 "data: 36.5"
    def sensor_body_temperature_callback(self,msg):
        #self.interaction_pub.publish(msg.data) # Forwarding the patient EPC code to the interaction manager
        global sensor_data_msg_state_machine
        sensor_data_msg_state_machine.body_temperature = msg.data
        rospy.loginfo(f"Body Temperature: {msg.data}")

        self.received_data = True

    def execute(self, userdata):
        self.interaction_complete = False
        self.received_data = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)

        self.motion_pub.publish("wake_up")

        rospy.loginfo("Measuring body temperature...")
        #self.motion_pub.publish("raise_left_arm")
        self.audio_pub.publish("Bring your wrist close to the reader to collect your temperature")
        self.sensor_pub.publish("body_temperature")

        while not rospy.is_shutdown() and not self.received_data:
            rospy.loginfo("Waiting to collect data") 
            rospy.sleep(5)
        
        global jetson_ip
        global sensor_data_msg
        self.screen_pub.publish(f"http://{jetson_ip}/DISPLAY_BODY_TEMPERATURE?temp={sensor_data_msg_state_machine.body_temperature}")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for next page to load")
            rospy.sleep(5)
        
        if self.next_state == "DISPLAY_BODY_TEMPERATURE":
            return 'next'
        else:
            return 'skip'


class DisplayBodyTemperature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False

    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "MEASURE_HEART_RATE_AND_SPO2_INSTRUCTION" or msg.data == "MEASURE_BLOOD_PRESSURE_INSTRUCTION":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True
    
    def execute(self, userdata):
        self.interaction_complete = False
        self.motion_pub.publish("wake_up")

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("lower_right_arm")
        self.audio_pub.publish("Your body temperature is displayed below. If you are happy and wish to move to the measurement of your heart rate and SPO2. Press the skip button if you would like to skip this test.")
        rospy.loginfo("Body temperature reading complete.")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)

        if self.next_state == "MEASURE_HEART_RATE_AND_SPO2_INSTRUCTION":
            return 'next'
        else:
            return 'skip'

class MeasureHeartRateAndSPO2Instruction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
        
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "MEASURE_HEART_RATE_AND_SPO2" or msg.data == "MEASURE_BLOOD_PRESSURE_INSTRUCTION":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True
    
    
    def execute(self, userdata):
        self.interaction_complete = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")

        self.audio_pub.publish("Please view the video to get instructions on how to collect your heart rate and blood oxygen level. When you have gone through the instructions please press next to go over to the scanning.")
     
        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)

        if self.next_state == "MEASURE_HEART_RATE_AND_SPO2":
            return 'next'
        else:
            return 'skip'

class MeasureHeartRateAndSPO2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.sensor_pub = rospy.Publisher('/sensor_manager/sensor_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)

        self.sensor_heart_rate_sub = rospy.Subscriber('/sensor_manager/heart_rate_and_spo2', heart_rate_and_spo2_msg, self.sensor_heart_rate_and_spo2_callback)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
        self.received_data = False

    
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "DISPLAY_HEART_RATE_AND_SPO2" or msg.data == "MEASURE_BLOOD_PRESSURE_INSTRUCTION":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True


#rostopic pub -1 /sensor_manager/heart_rate_and_spo2 hope_msgs/heart_rate_and_spo2_msg 80 98
    def sensor_heart_rate_and_spo2_callback(self,msg):
        #self.interaction_pub.publish(msg.data)
        rospy.loginfo(f"Heart Rate: {msg.heart_rate}. SPO2: {msg.spo2}")
        global sensor_data_msg_state_machine
        sensor_data_msg_state_machine.heart_rate = msg.heart_rate
        sensor_data_msg_state_machine.spo2 = msg.spo2

        self.received_data = True
    
    def execute(self, userdata):
        self.interaction_complete = False
        self.received_data = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")

        rospy.loginfo("Measuring heart rate and SPO2...")
        self.motion_pub.publish("raise_left_arm")
        self.audio_pub.publish("Please follow the instructions shown previously to collect your heart rate and blood oxygen level.")
        self.sensor_pub.publish("heart_rate_and_spo2")

        while not rospy.is_shutdown() and not self.received_data:
            rospy.loginfo("Waiting to collect data")
            rospy.sleep(5)
        
        global jetson_ip
        global sensor_data_msg
        self.screen_pub.publish(f"http://{jetson_ip}/DISPLAY_HEART_RATE_AND_SPO2?heart_rate={sensor_data_msg_state_machine.heart_rate}&spo2={sensor_data_msg_state_machine.spo2}")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for next page to load")
            rospy.sleep(5)
        
        if self.next_state == "DISPLAY_HEART_RATE_AND_SPO2":
            return 'next'
        else:
            return 'skip'


class DisplayHeartRateAndSPO2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
    
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "MEASURE_BLOOD_PRESSURE_INSTRUCTION" or msg.data == "MEASURE_ECG_INSTRUCTION":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True

    
    def execute(self, userdata):
        self.interaction_complete = False
        
        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")

        self.motion_pub.publish("lower_left_arm")
        self.audio_pub.publish("Your heart rate and SPO2 is displayed below. If you are happy and wish to move to the measurement of your blood pressure, please press the start button. If you would like to skip this data collection press skip.")
        rospy.loginfo("HR/SPO2 reading complete.")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)

        if self.next_state == "MEASURE_BLOOD_PRESSURE_INSTRUCTION":
            return 'next'
        else:
            return 'skip'

class MeasureBloodPressureInstruction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
        
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "MEASURE_BLOOD_PRESSURE" or msg.data == "MEASURE_ECG_INSTRUCTION":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True

    
    def execute(self, userdata):
        self.interaction_complete = False
        
        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")
        self.audio_pub.publish("Please view the video to get instructions on how to collect your Blood Pressure. When you have gone through the instructions please press next to input the data.")
     
        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)

        if self.next_state == "MEASURE_BLOOD_PRESSURE":
            return 'next'
        else:
            return 'skip'

class MeasureBloodPressure(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        #Blood pressure is unique because its data comes from the interaction server itself instead of the sensor manager. Hence the state machine is the subscriber to the interaction manager
        self.sensor_blood_pressure_pub = rospy.Subscriber('/sensor_manager/blood_pressure', blood_pressure_msg, self.get_user_input_blood_pressure_callback)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
    
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "MEASURE_ECG_INSTRUCTION":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True
   

    def get_user_input_blood_pressure_callback(self,msg):
        global sensor_data_msg_state_machine
        sensor_data_msg_state_machine.systolic = msg.systolic
        sensor_data_msg_state_machine.diastolic = msg.diastolic

        rospy.loginfo(f"Blood pressure (sys): {msg.systolic}")
        rospy.loginfo(f"Blood pressure (dia): {msg.diastolic}")
        # Can do more here with processing the blood pressure data
    
    def execute(self, userdata):
        self.interaction_complete = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")
        self.audio_pub.publish("Please follow the instructions shown previously to collect your blood pressure. Please input the data into the fields shown below.")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for measurement to complete")
            rospy.sleep(5)
        
        return 'next'


class MeasureECGInstruction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
        
    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "MEASURE_ECG" or msg.data == "UPLOAD_SENSOR_DATA":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True

    
    def execute(self, userdata):
        self.interaction_complete = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")
        self.audio_pub.publish("Please view the video to get instructions on how to collect your ECG data. When you have completed the attaching the electrodes please press next to go over to the scanning.")
     
        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)

        if self.next_state == "MEASURE_ECG":
            return 'next'
        else:
            return 'skip'

class MeasureECG(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['next'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__
        
        self.sensor_pub = rospy.Publisher('/sensor_manager/sensor_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)

        self.sensor_ecg_sub = rospy.Subscriber('/sensor_manager/ecg', ecg_data_msg, self.sensor_ecg_callback)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', ecg_data_msg, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
        self.received_data = False

#rostopic pub -1 /sensor_manager/ecg hope_msgs/ecg_data_msg "data_arr: [1,2,3,4,5,6,7,8,9,10]"
    def sensor_ecg_callback(self,msg):
        global sensor_data_msg_state_machine
        sensor_data_msg_state_machine.ecg_data = msg.data_arr
        rospy.loginfo("Recieved ECG data")
        self.received_data = True

    def interaction_callback(self, msg):
        
        self.next_state = msg.data
        if msg.data == "UPLOAD_SENSOR_DATA":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True


    def execute(self, userdata):
        self.interaction_complete = False
        self.received_data = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")
        rospy.loginfo("Measuring ECG...")
        self.audio_pub.publish("Please hold still for 20 seconds while we collect your ECG data.")
        self.sensor_pub.publish("ecg")

        while not rospy.is_shutdown() and not self.received_data:
            rospy.loginfo("Waiting to collect data")
            rospy.sleep(5)
        
        rospy.loginfo("ECG data collected.")

        global jetson_ip
        self.screen_pub.publish(f"http://{jetson_ip}/UPLOAD_SENSOR_DATA")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)

        return 'next'


class UploadSensorData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.server_pub = rospy.Publisher('/server_manager/upload_sensor_data', sensor_data_msg, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/server_manager/upload_status', Bool, self.upload_status_callback)
        
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False

        self.upload_complete = False

    def interaction_callback(self, msg):
        
        if msg.data == "ASK_PRESET_QUESTIONS_SETUP":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True
   

    def upload_status_callback(self, msg):
        global sensor_data_msg
        if msg.data:
            rospy.loginfo("Data upload successful.")
            self.upload_complete = True
        else:
            rospy.loginfo("Data upload failed.")
            self.server_pub.publish(sensor_data_msg)

    def execute(self, userdata):
        self.upload_complete = False
        self.interaction_complete = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")
        rospy.loginfo("Uploading data to the server...")
        self.audio_pub.publish("Please wait while we upload your test results to the server.")
        
        global sensor_data_msg_state_machine
        self.server_pub.publish(sensor_data_msg_state_machine)

        # while not rospy.is_shutdown() and not self.upload_complete:
        #     rospy.loginfo("Waiting for data to upload")
        #     rospy.sleep(5)

        rospy.sleep(5)

        rospy.loginfo("Data upload completed.")
        self.audio_pub.publish("Data upload completed")

        global jetson_ip
        self.screen_pub.publish(f"http://{jetson_ip}/ASK_PRESET_QUESTIONS_SETUP")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for screen to load")
            rospy.sleep(5)

        return 'next'


class AskPresetQuestionsSetup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])

        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.audio_speak_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
    
    def interaction_callback(self, msg):
        self.next_state = msg.data
        if msg.data == "ASK_PRESET_QUESTIONS" or msg.data == "THANK_PATIENT":
            rospy.loginfo(f"Interaction state: {msg.data}")
            self.interaction_complete = True

    def execute(self, userdata):
        self.interaction_complete = False
        self.recieved_questions = False

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)

        self.motion_pub.publish("wake_up")

        self.audio_speak_pub.publish("Please answer the following questions set by your doctor. If you would like the question to be repeated, please say repeat the question. Please press the start button to begin. You may also choose to skip this section")
        rospy.sleep(5)

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(2)

        if self.next_state == "ASK_PRESET_QUESTIONS":
            return 'next'
        else:
            return 'skip'


class AskPresetQuestions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        self.audio_speak_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.audio_record_pub = rospy.Publisher('/audio_manager/start_transcribe', Bool, queue_size=10)

        self.interaction_questions_answered_pub = rospy.Publisher('/server_manager/question_details_answered', string_arr_msg, queue_size=10)
        self.interaction_questions_details_sub = rospy.Subscriber('/server_manager/question_details', string_arr_msg, self.interaction_questions_details_callback)

        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)

        self.audio_text_from_speech_sub = rospy.Subscriber('/audio_manager/transcribed_text', String, self.audio_text_from_speech_callback)
        self.interaction_state_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_state_callback)

        self.interaction_complete = False
        self.received_question_details = False
        self.received_question_answer = False
        self.current_question_number = 0

    def interaction_questions_details_callback(self, msg):
        self.questions_details_questions = msg.data
        self.questions_details_answers = [""] * len(msg.data)  # instatiate it as the same so the sizes are correct. The data will be overwritten anyway
        self.question_details_size = len(msg.data)
        rospy.loginfo(f"Number of questions: {self.question_details_size}")
        self.received_question_details = True
        rospy.loginfo("Recieved question details")
        rospy.loginfo(f"Questions: {self.questions_details_questions}")
    
    def audio_text_from_speech_callback(self, msg):
        if msg.data == "repeat the question":
            self.audio_speak_pub.publish(self.questions_details_questions[self.current_question_number])
        else:
            self.questions_details_answers[self.current_question_number] = msg.data
            rospy.loginfo(f"Answer to question {self.current_question_number}: {msg.data}")
            self.received_question_answer = True
    
    def interaction_state_callback(self, msg):
        self.next_state = msg.data
        if msg.data == "THANK_PATIENT":
            self.interaction_complete = True
   

    def execute(self, userdata):
        self.interaction_complete = False
        self.received_question_details = False
        self.received_question_answer = False
        self.audio_record_pub.publish(False)
        self.current_question_number = 0

        self.state_pub.publish(self.state_name)
        rospy.loginfo(f"In state: {self.state_name}")
        #self.motion_pub.publish("wake_up")

        while not rospy.is_shutdown() and not self.received_question_details:
            rospy.loginfo("Waiting for question details")
            rospy.sleep(5)
        
        for self.current_question_number in range(self.question_details_size):
            self.received_question_answer = False
            global jetson_ip

            self.screen_pub.publish(f"http://{jetson_ip}/ASK_PRESET_QUESTIONS?question={self.questions_details_questions[self.current_question_number].replace(' ', '+')}")
            self.audio_speak_pub.publish(self.questions_details_questions[self.current_question_number])
            rospy.sleep(3)
            self.audio_record_pub.publish(True)
            while not rospy.is_shutdown() and not self.received_question_answer:
                rospy.loginfo("Waiting for question answer")
                rospy.sleep(5)
            if self.current_question_number != self.question_details_size - 1:
                self.audio_speak_pub.publish("Thank you for your response. Moving onto the next question.")
            else:
                self.audio_speak_pub.publish("Thank you for your response. This was the final question")
            rospy.sleep(2)

        rospy.loginfo("All questions answered.")

        self.interaction_questions_answered_pub.publish(self.questions_details_answers)
        rospy.loginfo("Answers sent for uploading")
        self.audio_speak_pub.publish("Thank you for answering the questions. Your responses have been recorded and are being uploaded.")


        self.screen_pub.publish(f"http://{jetson_ip}/THANK_PATIENT")

        while not rospy.is_shutdown() and not self.interaction_complete:
            rospy.loginfo("Waiting for patient to respond")
            rospy.sleep(5)
        
        return 'next'


class ThankPatient(smach.State):
    def __init__(self):
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__

        smach.State.__init__(self, outcomes=['navigate_home'])
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)
        self.interaction_sub = rospy.Subscriber('/interaction_manager/current_interaction_state', String, self.interaction_callback, queue_size=1)
        self.interaction_complete = False
    
    def interaction_callback(self, msg):
        
        if msg.data == "IDLE":
            self.interaction_complete = True
  

    def execute(self, userdata):
        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.motion_pub.publish("wake_up")
        self.audio_pub.publish("Thank you for your time. Your vitals have been recorded. Your responses will be uploaded shortly. Bye bye")
        rospy.sleep(10)
        self.screen_pub.publish(f"http://{jetson_ip}/IDLE")


        return 'navigate_home'

class NavigateBackHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)
        self.state_name = self.__class__.__name__
        self.motion_pub = rospy.Publisher('/motion_manager/motion_command', String, queue_size=10)
        self.nav_goal_pub = rospy.Publisher('/nav_module/goal', String, queue_size=10)
        self.nav_status_sub = rospy.Subscriber('/nav_module/status', String, self.nav_status_callback)
        self.screen_pub = rospy.Publisher('/screen_manager/url_load', String, queue_size=10)
        self.goal_reached = False
    
    def nav_status_callback(self, msg):
        if msg.data == "Goal Reached":
            self.goal_reached = True

    def execute(self, userdata):
        self.goal_reached = False
        self.motion_pub.publish("wake_up")
        self.motion_pub.publish("face_track_off")
        self.screen_pub.publish(f"http://{jetson_ip}/IDLE")

        rospy.loginfo(f"In state: {self.state_name}")
        self.state_pub.publish(self.state_name)
        self.nav_goal_pub.publish("home")

        while not rospy.is_shutdown() and not self.goal_reached:
            rospy.loginfo("Navigating back home...")
            rospy.sleep(2)
        
        rospy.loginfo("Reached home.")

        return 'next'

# -------------------------
# Main Function
# -------------------------

def main():
    ip = "192.168.1.101"
    username = "pepper"
    password = "BioART123"

    rospy.init_node('state_machine_node')

    # Create a top-level state machine
    sm = smach.StateMachine(outcomes=['loop'])

    global sensor_data_msg_state_machine 
    sensor_data_msg_state_machine = sensor_data_msg()

    with sm:
        # Add states to the state machine
        smach.StateMachine.add('IDLE', Idle(), transitions={'read_patient_queue': 'READ_PATIENT_QUEUE'})
        
        smach.StateMachine.add('READ_PATIENT_QUEUE', ReadPatientQueue(),
                               transitions={'sanitised': 'TRAVEL_TO_PATIENT',
                                            'alert_nurse': 'ALERT_NURSE_FOR_SANITATION'})

        smach.StateMachine.add('ALERT_NURSE_FOR_SANITATION', AlertNurseForSanitation(),
                               transitions={'sanitised': 'TRAVEL_TO_PATIENT'})

        smach.StateMachine.add('TRAVEL_TO_PATIENT', TravelToPatient(),
                               transitions={'start_interaction': 'START_INTERACTION'})

        smach.StateMachine.add('START_INTERACTION', StartInteraction(),
                               transitions={'next': 'MEASURE_BODY_TEMPERATURE',
                                            'exit_interaction': 'THANK_PATIENT'})
                                            
        smach.StateMachine.add('SCAN_PATIENT_ID', ScanPatientID(),
                               transitions={'next': 'DISPLAY_PATIENT_INFO',
                                            'exit_interaction': 'THANK_PATIENT'})

        smach.StateMachine.add('DISPLAY_PATIENT_INFO', DisplayPatientInfo(),
                               transitions={'next': 'MEASURE_BODY_TEMPERATURE',
                                            'exit_interaction': 'THANK_PATIENT'})

        smach.StateMachine.add('MEASURE_BODY_TEMPERATURE', MeasureBodyTemperature(),
                               transitions={'next': 'DISPLAY_BODY_TEMPERATURE',
                                            'skip': 'MEASURE_HEART_RATE_AND_SPO2_INSTRUCTION'})  # Skip to next sensor's instruction

        smach.StateMachine.add('DISPLAY_BODY_TEMPERATURE', DisplayBodyTemperature(),
                               transitions={'next': 'MEASURE_HEART_RATE_AND_SPO2_INSTRUCTION',
                                            'skip': 'MEASURE_BLOOD_PRESSURE_INSTRUCTION'})  # Skip to next sensor's instruction

        smach.StateMachine.add('MEASURE_HEART_RATE_AND_SPO2_INSTRUCTION', MeasureHeartRateAndSPO2Instruction(),
                               transitions={'next': 'MEASURE_HEART_RATE_AND_SPO2',
                                            'skip': 'MEASURE_BLOOD_PRESSURE_INSTRUCTION'})  # Skip to next sensor's instruction

        smach.StateMachine.add('MEASURE_HEART_RATE_AND_SPO2', MeasureHeartRateAndSPO2(),
                               transitions={'next': 'DISPLAY_HEART_RATE_AND_SPO2',
                                            'skip': 'MEASURE_ECG_INSTRUCTION'})  # Skip to next sensor's instruction

        smach.StateMachine.add('DISPLAY_HEART_RATE_AND_SPO2', DisplayHeartRateAndSPO2(),
                                 transitions={'next': 'MEASURE_BLOOD_PRESSURE_INSTRUCTION', 
                                              'skip': 'MEASURE_ECG_INSTRUCTION'})
        
        smach.StateMachine.add('MEASURE_BLOOD_PRESSURE_INSTRUCTION', MeasureBloodPressureInstruction(),
                               transitions={'next': 'MEASURE_BLOOD_PRESSURE',
                                            'skip': 'MEASURE_ECG_INSTRUCTION'})  # Skip to next sensor's instruction

        smach.StateMachine.add('MEASURE_BLOOD_PRESSURE', MeasureBloodPressure(),
                               transitions={'next': 'MEASURE_ECG_INSTRUCTION'})  # Skip to questionnaire setup

        smach.StateMachine.add('MEASURE_ECG_INSTRUCTION', MeasureECGInstruction(),
                               transitions={'next': 'MEASURE_ECG',
                                            'skip': 'UPLOAD_SENSOR_DATA'})  # Skip to questionnaire setup

        smach.StateMachine.add('MEASURE_ECG', MeasureECG(),
                               transitions={'next': 'UPLOAD_SENSOR_DATA'})

        smach.StateMachine.add('UPLOAD_SENSOR_DATA', UploadSensorData(),
                               transitions={'next': 'ASK_PRESET_QUESTIONS_SETUP'})
        
        smach.StateMachine.add('ASK_PRESET_QUESTIONS_SETUP', AskPresetQuestionsSetup(),
                               transitions={'next': 'ASK_PRESET_QUESTIONS',
                                            'skip': 'THANK_PATIENT'})

        smach.StateMachine.add('ASK_PRESET_QUESTIONS', AskPresetQuestions(),
                               transitions={'next': 'THANK_PATIENT'})

        smach.StateMachine.add('THANK_PATIENT', ThankPatient(),
                               transitions={'navigate_home': 'NAVIGATE_BACK_HOME'})

        smach.StateMachine.add('NAVIGATE_BACK_HOME', NavigateBackHome(),
                               transitions={'next': 'IDLE'})  # Loop back to IDLE

    # Execute the state machine
    try:
        sm.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("State Machine interrupted. Shutting down.")
    finally:
        rospy.signal_shutdown("KeyboardInterrupt received")

if __name__ == '__main__':
    main()

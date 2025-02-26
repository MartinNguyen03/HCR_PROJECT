import rospy
import smach
import smach_ros
from std_msgs.msg import String
from std_msgs.msg import Bool

# Utility function for retries
def retry_logic(max_retries, check_function, retry_message):
    for attempt in range(max_retries):
        if check_function():
            return True
        rospy.loginfo(retry_message)
        rospy.sleep(1)
    return False

# -------------------------
# Pre-Patient States
# -------------------------

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['read_patient_queue'])
        self.state_pub = rospy.Publisher('/state_machine/status', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Robot is idle, waiting for a task...")
        self.state_pub.publish("Idle")
        rospy.sleep(1)
        return 'read_patient_queue'


class ReadPatientQueue(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sanitised', 'alert_nurse'])
        self.queue_pub = rospy.Publisher('/server_manager/visit_schedule', String, queue_size=10)
        self.queue_sub = rospy.Subscriber('/server_manager/visit_completed', Bool, self.queue_callback)
        self.sanitised = True  # Simulated sanitation check

    def queue_callback(self, msg):
        self.sanitised = msg.data

    def execute(self, userdata):
        rospy.loginfo("Reading patient queue...")
        self.queue_pub.publish("Requesting patient queue")
        rospy.sleep(1)

        if self.sanitised:
            rospy.loginfo("Sanitation confirmed.")
            return 'sanitised'
        else:
            rospy.loginfo("Sanitation required.")
            return 'alert_nurse'


class AlertNurseForSanitation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sanitised'])
        self.nurse_pub = rospy.Publisher('/state_machine/alert_nurse', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Alerting nurse for sanitation.")
        self.nurse_pub.publish("Alerting nurse for sanitation")
        rospy.sleep(2)
        rospy.loginfo("Sanitation completed.")
        return 'sanitised'


class TravelToPatient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_interaction'])
        self.nav_goal_pub = rospy.Publisher('/nav_module/goal', String, queue_size=10)
        self.nav_status_sub = rospy.Subscriber('/nav_module/status', String, self.nav_status_callback)
        self.goal_reached = False

    def nav_status_callback(self, msg):
        if msg.data == "Goal Reached":
            self.goal_reached = True

    def execute(self, userdata):
        rospy.loginfo("Travelling to the patient.")
        self.nav_goal_pub.publish("Patient Location")
        rospy.sleep(2)

        while not self.goal_reached:
            rospy.loginfo("Waiting to reach the patient...")
            rospy.sleep(1)

        rospy.loginfo("Reached the patient.")
        return 'start_interaction'


# -------------------------
# Interaction States
# -------------------------

class InitialInteraction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['permission_granted', 'alert_nurse'])
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Hello, I am HOPE, your medical assistant robot.")
        self.audio_pub.publish("Hello, I am HOPE, your medical assistant robot.")
        rospy.sleep(1)

        permission = True  # Simulated permission
        if permission:
            rospy.loginfo("Permission granted.")
            return 'permission_granted'
        else:
            rospy.loginfo("Permission denied.")
            return 'alert_nurse'


class AskPresetQuestions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['request_permission', 'alert_nurse'])
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("I have a few questions for you.")
        self.audio_pub.publish("I have a few questions for you.")
        rospy.sleep(2)

        emergency = False  # Simulated emergency check
        if emergency:
            rospy.loginfo("Concerning words detected.")
            return 'alert_nurse'
        else:
            rospy.loginfo("Questions completed.")
            return 'request_permission'


class RequestPermissionForSensorData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['permission_granted', 'alert_nurse'])
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Requesting permission to collect sensor data...")
        self.audio_pub.publish("Requesting permission to collect sensor data...")
        rospy.sleep(1)

        permission = True  # Simulated permission
        if permission:
            rospy.loginfo("Permission granted.")
            return 'permission_granted'
        else:
            rospy.loginfo("Permission denied.")
            return 'alert_nurse'

# -------------------------
# Data Collection States
# -------------------------

class RFIDTemperature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.sensor_pub = rospy.Publisher('/sensor_manager/sensor_command', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("I will now take your temperature.")
        self.sensor_pub.publish("Start RFID Temperature Measurement")
        rospy.sleep(2)

        success = retry_logic(3, lambda: rospy.get_param('/rfid_success', True), "Retrying RFID measurement...")
        if success:
            rospy.loginfo("Temperature reading complete.")
            return 'next'
        else:
            rospy.loginfo("Temperature data collection failed. Skipping.")
            return 'skip'
        
class HeartRateSPO2Collection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.sensor_pub = rospy.Publisher('/sensor_manager/sensor_command', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("I will now take your heart rate and SPO2 data.")
        self.sensor_pub.publish("Start Heart Rate and SPO2 Measurement")
        rospy.sleep(2)

        success = retry_logic(3, lambda: rospy.get_param('/heart_rate_and_spo2_success', True), "Retrying Heart rate and SPO2 measurement...")
        if success:
            rospy.loginfo("Heart rate and SPO2 reading complete.")
            return 'next'
        else:
            rospy.loginfo("Heart rate and SPO2 collection failed. Skipping.")
            return 'skip'
        
class BloodPressureCollection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.sensor_pub = rospy.Publisher('/sensor_manager/sensor_command', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("I will now take your blood pressure.")
        self.sensor_pub.publish("Start Blood Pressure Measurement")
        rospy.sleep(2)

        success = retry_logic(3, lambda: rospy.get_param('/blood_pressure_success', True), "Retrying Blood Pressure measurement...")
        if success:
            rospy.loginfo("Blood pressure reading complete.")
            return 'next'
        else:
            rospy.loginfo("Blood pressure data collection failed. Skipping.")
            return 'skip'
        
class ECGCollection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])
        self.sensor_pub = rospy.Publisher('/sensor_manager/sensor_command', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("I will now take your ECG data.")
        self.sensor_pub.publish("Start ECG Measurement")
        rospy.sleep(2)

        success = retry_logic(3, lambda: rospy.get_param('/ecg_success', True), "Retrying ECG measurement...")
        if success:
            rospy.loginfo("ECG reading complete.")
            return 'next'
        else:
            rospy.loginfo("ECG data collection failed. Skipping.")
            return 'skip'

# -------------------------
# Post Patient States
# -------------------------

class UploadData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['thank_patient'])
        self.server_pub = rospy.Publisher('/server_manager/upload_data', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Uploading data to the server...")
        self.server_pub.publish("Uploading patient data")
        rospy.sleep(2)
        rospy.loginfo("Data upload completed.")
        return 'thank_patient'

class ThankPatient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigate_home'])
        self.audio_pub = rospy.Publisher('/audio_manager/speech_command', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Thank you for your time.")
        self.audio_pub.publish("Thank you for your time.")
        rospy.sleep(1)
        return 'navigate_home'

class NavigateBackHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        print("Navigating back to the home station...")
        time.sleep(2)
        return 'done'

# -------------------------
# Main Function
# -------------------------

def main():
    rospy.init_node('state_machine_node')

    # Create a top-level state machine
    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        # Add states to the state machine
        smach.StateMachine.add('IDLE', Idle(), transitions={'read_patient_queue': 'READ_PATIENT_QUEUE'})
        smach.StateMachine.add('READ_PATIENT_QUEUE', ReadPatientQueue(),
                               transitions={'sanitised': 'TRAVEL_TO_PATIENT',
                                            'alert_nurse': 'ALERT_NURSE_FOR_SANITATION'})
        smach.StateMachine.add('ALERT_NURSE_FOR_SANITATION', AlertNurseForSanitation(),
                               transitions={'sanitised': 'TRAVEL_TO_PATIENT'})
        smach.StateMachine.add('TRAVEL_TO_PATIENT', TravelToPatient(),
                               transitions={'start_interaction': 'INITIAL_INTERACTION'})
        smach.StateMachine.add('INITIAL_INTERACTION', InitialInteraction(),
                               transitions={'permission_granted': 'ASK_PRESET_QUESTIONS',
                                            'alert_nurse': 'done'})
        smach.StateMachine.add('ASK_PRESET_QUESTIONS', AskPresetQuestions(),
                               transitions={'request_permission': 'REQUEST_PERMISSION_FOR_SENSOR_DATA',
                                            'alert_nurse': 'done'})
        smach.StateMachine.add('REQUEST_PERMISSION_FOR_SENSOR_DATA', RequestPermissionForSensorData(),
                               transitions={'permission_granted': 'RFID_TEMPERATURE',
                                            'alert_nurse': 'done'})
        smach.StateMachine.add('RFID_TEMPERATURE', RFIDTemperature(),
                               transitions={'next': 'UPLOAD_DATA',
                                            'skip': 'UPLOAD_DATA'})
        smach.StateMachine.add('HEART_RATE_AND_SPO2', HeartRateSPO2Collection(),
                               transitions={'next': 'BLOOD_PRESSURE'})
        smach.StateMachine.add('BLOOD_PRESSURE', BloodPressureCollection(),
                               transitions={'next': 'ECG'})
        smach.StateMachine.add('ECG', ECGCollection(),
                               transitions={'next': 'UPLOAD_DATA'})
        smach.StateMachine.add('UPLOAD_DATA', UploadData(),
                               transitions={'thank_patient': 'THANK_PATIENT'})
        smach.StateMachine.add('THANK_PATIENT', ThankPatient(),
                               transitions={'navigate_home': 'done'})
        smach.StateMachine.add('NAVIGATE_BACK_HOME', NavigateBackHome(),
                               transitions={'done': 'done'})
        
    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()

import smach
import smach_ros
import time

# Utility function for retries
def retry_logic(max_retries, check_function, retry_message):
    for attempt in range(max_retries):
        if check_function():
            return True
        print(retry_message)
        time.sleep(1)
    return False

# -------------------------
# Pre-Patient States
# -------------------------

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['read_patient_queue'])

    def execute(self, userdata):
        print("Robot is idle, waiting for a task...")
        time.sleep(1)
        return 'read_patient_queue'

class ReadPatientQueue(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sanitised', 'alert_nurse'])

    def execute(self, userdata):
        print("Reading patient queue...")
        time.sleep(1)
        sanitised = True  # Simulated sanitation check
        if sanitised:
            print("Sanitation confirmed.")
            return 'sanitised'
        else:
            print("Sanitation required.")
            return 'alert_nurse'

class AlertNurseForSanitation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sanitised'])

    def execute(self, userdata):
        print("Alerting nurse for sanitation...")
        time.sleep(2)
        print("Sanitation completed.")
        return 'sanitised'

class TravelToPatient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_interaction'])

    def execute(self, userdata):
        print("Travelling to the patient...")
        time.sleep(2)
        return 'start_interaction'

# -------------------------
# Interaction States
# -------------------------

class InitialInteraction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['permission_granted', 'alert_nurse'])

    def execute(self, userdata):
        print("Hello, I am HOPE, your medical assistant robot.")
        time.sleep(1)
        permission = True  # Simulated permission
        if permission:
            print("Permision granted.")
            return 'permission_granted'
        else:
            print("Permission denied.")
            return 'alert_nurse'

class AskPresetQuestions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['request_permission', 'alert_nurse'])
    
    def execute(self, userdata):
        print("I have a few questions for you.")
        time.sleep(2)
        emergency = False  # Simulated emergency check
        if emergency:
            print("Concerning words detected.")
            return 'alert_nurse'
        else:
            print("Questions completed.")
            return 'request_permission'

class RequestPermissionForSensorData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['permission_granted', 'alert_nurse'])

    def execute(self, userdata):
        print("Requesting permission to collect sensor data...")
        time.sleep(1)
        permission = True  # Simulated permission
        if permission:
            print("Permission granted.")
            return 'permission_granted'
        else:
            print("Permission denied.")
            return 'alert_nurse'

# -------------------------
# Data Collection States
# -------------------------

class RFIDTemperature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])

    def execute(self, userdata):
        print("I will now take your temperature.'")
        ready = input("Are you ready?").lower() == "yes"
        if not ready:
            skip = input("Would you like to skip this step? (yes/no): ").lower() == "yes"
            if skip:
                return 'skip'
            else:
                print("Waiting for the patient to get ready...")
                time.sleep(2)

        print("Checking if RFID tag is readable...")
        if not retry_logic(3, lambda: input("Is RFID readable? (yes/no): ").lower() == "yes",
                           "Please adjust your arm position."):
            print("Failed to collect temperature data after 3 attempts.")
            return 'skip'

        print("Collecting temperature data...")
        if not retry_logic(3, lambda: input("Does the collected data make sense? (yes/no): ").lower() == "yes",
                           "Please retry temperature collection."):
            print("Temperature data collection failed. Skipping.")
            return 'skip'

        print("Temperature reading complete.")
        return 'next'

class HeartRateSPO2Collection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])

    def execute(self, userdata):
        print("I will now take your heart rate and SP02.")
        ready = input("Are you ready? (yes/no): ").lower() == "yes"
        if not ready:
            skip = input("Would you like to skip this step? (yes/no): ").lower() == "yes"
            if skip:
                return 'skip'
            else:
                print("Waiting for the patient to get ready...")
                time.sleep(2)

        print("Taking heart rate and SPO2 measurement...")
        time.sleep(2)
        if not retry_logic(3, lambda: input("Does the collected data make sense? (yes/no): ").lower() == "yes",
                           "Please retry SPO2 and heart rate measurement."):
            print("Heart rate and SPO2 collection failed. Skipping.")
            return 'skip'

        print("Heart rate and SPO2 reading completed.")
        return 'next'

class BloodPressureCollection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])

    def execute(self, userdata):
        print("I will now measure your blood pressure.")
        ready = input("Are you ready? (yes/no): ").lower() == "yes"
        if not ready:
            skip = input("Would you like to skip this step? (yes/no): ").lower() == "yes"
            if skip:
                return 'skip'
            else:
                print("Waiting for the patient to get ready...")
                time.sleep(2)

        print("Showing demo video...")
        understands = input("Do you understand the process? (yes/no): ").lower() == "yes"
        if not understands:
            print("Replaying demo video...")
            time.sleep(3)
            understands = input("Do you now understand the process? (yes/no): ").lower() == "yes"
            if not understands:
                print("Skipping blood pressure collection.")
                return 'skip'

        print("Please pick up the BP sensor and input values...")
        if not retry_logic(3, lambda: input("Are the input values valid? (yes/no): ").lower() == "yes",
                           "Please retry entering BP values."):
            print("Blood pressure data collection failed. Skipping.")
            return 'skip'

        print("Blood pressure reading completed.")
        return 'next'

class ECGCollection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next', 'skip'])

    def execute(self, userdata):
        print("Communicating with patient: 'We will now measure your heart’s electrical activity.'")
        ready = input("Is the patient ready? (yes/no): ").lower() == "yes"
        if not ready:
            skip = input("Would the patient like to skip this step? (yes/no): ").lower() == "yes"
            if skip:
                return 'skip'
            else:
                print("Waiting for the patient to get ready...")
                time.sleep(2)

        print("Showing demo video...")
        understands = input("Does the patient understand the process? (yes/no): ").lower() == "yes"
        if not understands:
            print("Replaying demo video...")
            time.sleep(3)
            understands = input("Does the patient now understand the process? (yes/no): ").lower() == "yes"
            if not understands:
                print("Skipping ECG collection.")
                return 'skip'

        print("Please pick up the electrode and starting ECG data collection...")
        time.sleep(4)
        if not retry_logic(3, lambda: input("Does the collected ECG data make sense? (yes/no): ").lower() == "yes",
                           "Please retry ECG data collection."):
            print("ECG data collection failed. Skipping.")
            return 'skip'

        print("ECG data collection complete. Asking patient to remove the electrode.")
        return 'complete'

# -------------------------
# Post Patient States
# -------------------------

class UploadData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['thank_patient'])

    def execute(self, userdata):
        print("Uploading data to the server...")
        time.sleep(2)
        print("Data upload completed.")
        return 'thank_patient'

class ThankPatient(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigate_home'])

    def execute(self, userdata):
        print("Thank you for your time.")
        time.sleep(1)
        return 'navigate_home'

# Define state: Navigate Back Home
class NavigateBackHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        print("Navigating back to the home station...")
        time.sleep(2)
        return 'done'
    

# Main function to execute the state machine
def main():
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
                               transitions={'next': 'HEART_RATE_AND_SPO2'})
        smach.StateMachine.add('HEART_RATE_AND_SPO2', HeartRateSPO2Collection(),
                               transitions={'next': 'BLOOD_PRESSURE'})
        smach.StateMachine.add('BLOOD_PRESSURE', BloodPressureCollection(),
                               transitions={'next': 'ECG'})
        smach.StateMachine.add('ECG', ECGCollection(),
                               transitions={'next': 'UPLOAD_DATA'})
        smach.StateMachine.add('UPLOAD_DATA', UploadData(),
                               transitions={'thank_patient': 'THANK_PATIENT'})
        smach.StateMachine.add('THANK_PATIENT', ThankPatient(),
                               transitions={'navigate_home': 'NAVIGATE_BACK_HOME'})
        smach.StateMachine.add('NAVIGATE_BACK_HOME', NavigateBackHome(),
                               transitions={'done': 'done'})

    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()

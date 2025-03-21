
#need to add the following libraries to the requirements.txt file
import depthai as dai, cv2, face_recognition, numpy as np, requests

def send_ready(): 
    print("Ready to start")
def start_navigation(): 
    print("Navigation started")
    
p_enc = face_recognition.face_encodings(face_recognition.load_image_file("patient.jpg"))[0]
p = dai.Pipeline()
c = p.createColorCamera()
c.setPreviewSize(300,300)
d = p.createMobileNetDetectionNetwork()
c.preview.link(d.input)
d.setConfidenceThreshold(0.5)
command_received = False

with dai.Device(p) as dev:
 q = dev.getOutputQueue(name="detections", maxSize=4, blocking=False)
 pr = dev.getOutputQueue(name="preview", maxSize=4, blocking=False)
 while True:
  if not command_received:
   if input("Enter command: ")=="start": #add code to receive input from something else
    command_received = True
  f = np.array(pr.get().getCvFrame())
  for det in q.get().detections:
   x, y, w, h = int(det.x*300), int(det.y*300), int(det.width*300), int(det.height*300)
   face = f[y:y+h, x:x+w]
   enc = face_recognition.face_encodings(face)
   if enc and face_recognition.compare_faces([p_enc], enc[0])[0]:
    send_ready()
    if command_received: start_navigation()

#!/usr/bin/env python3
import rospy
import whisper
import pyaudio
import numpy as np
import wave
import io
from std_msgs.msg import String

class WhisperSTTNode:
    def __init__(self):
        rospy.init_node('sst_node', anonymous=True)
        self.text_pub = rospy.Publisher('/sst_text', String, queue_size=10)

        # Load Whisper model (you can change the model size)
        self.model = whisper.load_model("base")

        # Audio recording parameters
        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16  # 16-bit PCM
        self.channels = 1  # Mono
        self.rate = 16000  # Sampling rate (must match Whisper’s expected input)
        self.record_seconds = 5  # Duration of each audio capture

        self.audio_interface = pyaudio.PyAudio()
        rospy.loginfo("Whisper STT Node Initialized. Listening for audio...")

    def record_audio(self):
        """Records audio from the microphone."""
        stream = self.audio_interface.open(format=self.format,
                                           channels=self.channels,
                                           rate=self.rate,
                                           input=True,
                                           frames_per_buffer=self.chunk)
        rospy.loginfo("Recording...")
        frames = []

        for _ in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)

        rospy.loginfo("Recording finished.")
        stream.stop_stream()
        stream.close()

        return frames

    def transcribe_audio(self, frames):
        """Converts recorded audio frames to text using Whisper."""
        with io.BytesIO() as audio_buffer:
            wf = wave.open(audio_buffer, 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio_interface.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            audio_buffer.seek(0)

            # Transcribe using Whisper
            result = self.model.transcribe(audio_buffer)
            text = result["text"]

            return text

    def run(self):
        """Continuously captures and transcribes audio."""
        while not rospy.is_shutdown():
            frames = self.record_audio()
            transcribed_text = self.transcribe_audio(frames)

            # Publish the transcribed text
            self.text_pub.publish(transcribed_text)
            rospy.loginfo(f"Transcribed Text: {transcribed_text}")

if __name__ == '__main__':
    try:
        node = WhisperSTTNode()
        node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

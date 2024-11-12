#!/usr/bin/env python3

import os
import json
import string
import tempfile
import wave
import sounddevice as sd
import whisper
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

class WhisperSpeechRecognition(Node):
    def __init__(self):
        # Set the path to your VOSK model
        super().__init__("Whisper_Speech_Recognition")
        self.pub_audio = self.create_publisher(String, '/speech_to_text', 10)

        self.model = whisper.load_model("base.en")
 
        self.process_audio()

    def save_audio_to_tempfile(self, audio_data):
        tmpfile = tempfile.NamedTemporaryFile(delete=False, suffix='.wav', prefix='audio_', dir='.')
        with wave.open(tmpfile.name, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono audio
            wav_file.setsampwidth(2)  # 16-bit audio
            wav_file.setframerate(16000)  # Sample rate
            wav_file.writeframes(audio_data)
        return tmpfile.name

    def transcribe_audio(self, audio_data):
        audio_file = self.save_audio_to_tempfile(audio_data)
        try:
            result = self.model.transcribe(audio_file, language="en", word_timestamps=True)
            return result
        finally:
            os.remove(audio_file)
 
    def process_audio(self):
        accumulated_audio = bytearray()  # Accumulate current 5-second audio chunk
        previous_audio = bytearray()  # Store previous 5-second audio chunk
        previous_text = ""  # Store previous 5-second transcription

        with sd.InputStream(dtype='int16', channels=1, samplerate=16000, blocksize=80000) as stream:
            self.get_logger().info("Recording... Press Ctrl+C to stop.")
            while rclpy.ok():
                start_time_ns_5s = self.get_clock().now().nanoseconds
                start_time_ns_10s = start_time_ns_5s - 5e9
                indata, overflowed = stream.read(80000)  # 5 seconds at 16 kHz

                if overflowed:
                    self.get_logger().warning("Warning: Audio buffer overflowed!")
                
                # Transcribe the new 5-second audio chunk
                accumulated_audio.extend(indata)
                current = self.transcribe_audio(accumulated_audio)
                current_text = current["text"].translate(str.maketrans('', '', string.punctuation)).lower().strip()
                previous_text = current_text
                self.get_logger().info(f"Current 5-second transcription:{current_text}")

                if current_text != "":
                    current["start_time_ns"] = start_time_ns_5s
                    current["time_span"] = 5
                    current_json = json.dumps(current)
                    msg = String()
                    msg.data = json.dumps(current, indent=2)
                    self.pub_audio.publish(msg)

                # Combine with the previous chunk to form a 10-second audio
                combined_audio = previous_audio + accumulated_audio
                combined = self.transcribe_audio(combined_audio)
                combined_text = combined["text"].translate(str.maketrans('', '', string.punctuation)).lower().strip()

                self.get_logger().info(f"Combined 10-second transcription: {combined_text}")

                if combined_text != "" and previous_text != combined_text:
                    combined["start_time_ns"] = start_time_ns_10s
                    combined["time_span"] = 10
                    combined_json = json.dumps(combined)
                    msg = String()
                    msg.data = combined_json
                    self.pub_audio.publish(msg)

                previous_audio = accumulated_audio
                accumulated_audio = bytearray()

def main(args=None):
    rclpy.init(args=args)
    recognizer = WhisperSpeechRecognition()
    rclpy.spin(recognizer)
    rclpy.shutdown()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import os
import sys
import json
import queue
import vosk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
 
class VoskSpeechRecognition(Node):
    def __init__(self):
        # Set the path to your VOSK model
        super().__init__("vosk_speech_recognition")
        self.pub_audio = self.create_publisher(String, '/audio', 10)

        model_path = "/home/jake/ROSHome/ros2_ws/models/vosk-model-en-us-0.42-gigaspeech"
        self.q = queue.Queue()
 
        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            print('No input device found')
            raise ValueError('No input device found, device number == -1')
 
        device_info = sd.query_devices(self.input_dev_num, 'input')
        self.samplerate = int(device_info['default_samplerate'])
 
        self.model = vosk.Model(model_path)
        self.speech_recognize()

    def stream_callback(self, indata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))
 
    def speech_recognize(self):
        try:
            with sd.RawInputStream(samplerate=self.samplerate, blocksize=16000, device=self.input_dev_num, dtype='int16',
                                   channels=1, callback=self.stream_callback):
                print('Started recording')
                rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                rec.SetMaxAlternatives(5)
                print("Vosk is ready to listen!")
                while True:
                    data = self.q.get()
                    if rec.AcceptWaveform(data):
                        result = rec.FinalResult()
                        print(result)
                        result_dict = json.loads(result)
                        text = result_dict["alternatives"][0]["text"]
                        if len(text) > 2:
                            print("\033[91mFinal result:\033[0m", text)
                            msg = String()
                            msg.data = text
                            self.pub_audio.publish(msg)
                        rec.Reset()
                    else:
                        partial_result = rec.PartialResult()
                        if len(partial_result) > 20:
                            partial_dict = json.loads(partial_result)
                            #print("Partial result:", partial_dict["partial"])
 
        except KeyboardInterrupt as e:
            print("\nStopping the VOSK speech recognition...")
            raise e
        except Exception as e:
            print(f"An error occurred: {type(e).__name__}: {str(e)}")
 

def main(args=None):
    rclpy.init(args=args)
    recognizer = VoskSpeechRecognition()
    recognizer.speech_recognize()
    rclpy.spin(recognizer)
    rclpy.shutdown()


# if __name__ == '__main__':
#     recognizer = VoskSpeechRecognition()
#     recognizer.speech_recognize()
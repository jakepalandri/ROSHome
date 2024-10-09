#!/usr/bin/env python3

# Use default sample rate setting on the machine
# Publishes to the topic /audio AudioData message

import sys
import sounddevice as sd
import rclpy
from rclpy.node import Node
from time import sleep
from audio_messages.msg import AudioData

class AudioPublisher(Node):
    def __init__(self):
        super().__init__("audio_publisher")
        self.pub_audio = self.create_publisher(AudioData, '/audio', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.speech_recognize)

        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            self.get_logger().info('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(self.input_dev_num, 'input')
        # soundfile expects an int, sounddevice provides a float:
        
        self.samplerate = int(device_info['default_samplerate'])
        
        print(f"input sample rate {self.samplerate}")
        self.speech_recognize()        
    
    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        d = AudioData()

        d.data = bytes(indata)
        self.pub_audio.publish(d)
        

    def speech_recognize(self):
        try:
            with sd.RawInputStream(samplerate=self.samplerate,
                                   blocksize=16000,
                                   device=self.input_dev_num,
                                   dtype='int16',
                                   channels=1,
                                   callback=self.stream_callback):
                self.get_logger().info('Started recording')
                
                print("Start capture audio!")

                while(True):
                    print("I am Alive")
                    sleep(300)


        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))

def main(args=None):
    rclpy.init(args=args)
    audio_publisher = AudioPublisher()
    rclpy.spin(audio_publisher)
    audio_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
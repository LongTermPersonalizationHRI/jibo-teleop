"""
This is a helper class that handles the audio recording and sending it to SpeechAce
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, wrong-import-order

import _thread as thread
import binascii
import json
import subprocess
import time
import wave
import math
import pyaudio
import sys
from six.moves import queue

import rospy

USE_USB_MIC = True
MAX_BUFFER_SIZE_BEFORE_DISCARD = 590 # If buffer exceeds 50M and we aren't recording, dump and start over


class AudioRecorder:
    """
    Helper class that handles audio recording, converting to wav, and sending to SpeechAce
    """

    # CONSTANTS
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 48000

    CHUNK = 512 #16000
    #EXTERNAL_MIC_NAME = 'USB audio CODEC: Audio (hw:1,0)'
    EXTERNAL_MIC_NAME = 'Ensoniq AudioPCI: ES1371 DAC2/ADC (hw:0,0)'


    def __init__(self):
        # True if the phone is currently recording
        self.is_recording = False

        # Holds the audio data that turns into a wav file
        # Needed so that the data will be saved when recording audio in the new thread
        self.buffered_audio_data = []

        # Audio Subscriber node
        self.sub_audio = None

        # placeholder variable so we can see how long we recorded for
        self.start_recording_time = 0
        
        if USE_USB_MIC: #start recording so we dont have to repoen a new stream every time
            thread.start_new_thread(self.start_audio_stream, ())


    def start_audio_stream(self):
        mic_index = None
        audio = pyaudio.PyAudio()
        info = audio.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')

        #print(numdevices)
        #print("# of devices")

        for i in range(0, numdevices):

            #print(audio.get_device_info_by_host_api_device_index(0, i).get('name'))
            if (audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                print(audio.get_device_info_by_host_api_device_index(0, i).get('name'))
                if audio.get_device_info_by_host_api_device_index(0, i).get('name') == self.EXTERNAL_MIC_NAME:
                    mic_index = i
                    break

        if mic_index == None:
            print('NOT RECORDING, NO USB AUDIO DEVICE FOUND!')
            self.valid_recording = False
            pass
        else:
            # start Recording
            self.valid_recording = True
            print('USB Audio Device found, recording!')
            self.stream = audio.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE, input=True, frames_per_buffer=self.CHUNK, input_device_index=mic_index)

            # while self.is_recording:
            #     data = stream.read(self.CHUNK)
            #     buffered_audio_data.append(data)
            print(self.RATE)
            print(self.CHUNK)

            # Constantly add to buffered audio data stream 
            #TODO: publish on ros topic for bagging? 
            while(True):
                data = self.stream.read(self.CHUNK, exception_on_overflow=False)
                self.buffered_audio_data.append(data)
                #print(sys.getsizeof(self.buffered_audio_data))

                if (sys.getsizeof(self.buffered_audio_data) > MAX_BUFFER_SIZE_BEFORE_DISCARD) and not self.is_recording:
                    self.buffered_audio_data = []




    def record_usb_audio(self):

        while(self.is_recording):                
            data = self.stream.read(self.CHUNK, exception_on_overflow=False)
            self.buffered_audio_data.append(data)
        else: 
            time.sleep(1) #if configured to use USB Mic, but it doesn't exist, then just sleep

            # Stops the recording
            #stream.stop_stream()
            #stream.close()
            #audio.terminate()


    def start_recording(self, audio_filename):
        """
        Starts a new thread that records the microphone audio.
        """
        time.sleep(.8) #half second delay

        self.buffered_audio_data = []  # Resets audio data
        self.is_recording = True

        self.start_recording_time = time.time()

        #if self.valid_recording:

        #    thread.start_new_thread(self.record_usb_audio, ())


            
        

    def stop_recording(self, audio_filename):
        """
        ends the recording and makes the data into
        a wav file.
        """
        time.sleep(1) #one second delay
        self.is_recording = False  # Ends the recording


        wav_file = wave.open(audio_filename, 'wb')
        wav_file.setnchannels(AudioRecorder.CHANNELS)
        wav_file.setsampwidth(2)
        wav_file.setframerate(AudioRecorder.RATE)
        wav_file.writeframes(b''.join(self.buffered_audio_data))
        wav_file.close()

        elapsed_time = time.time() - self.start_recording_time
        print("recorded speech for " + str(elapsed_time) + " seconds")
        print('RECORDING SUCCESSFUL, writing to wav')

        time.sleep(.1)  # Gives time to return the data

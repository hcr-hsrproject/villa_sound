#!/usr/bin/python

from villa_audio.srv import *
from std_msgs.msg import String
import rospy

def handle_request_sound_transcript(req):
    requestSoundTranscriptResponse = RequestSoundTranscriptResponse()
    requestSoundTranscriptResponse.isGood = False
    try:
        speech_client = speech.Client()
        sms = SharedMicrophoneStream()
        audio_sample = speech_client.sample(
            stream=sms,
            encoding=speech.encoding.Encoding.LINEAR16,
            sample_rate_hertz=16000)
        alternatives = audio_sample.streaming_recognize('en-US')

        for alternative in alternatives:
            requestSoundTranscriptResponse.utterance = alternative.transcript
            requestSoundTranscriptResponse.isGood = True

    except RuntimeError:
        googleSpeech = cloud_speech_pb2.SpeechStub(
            make_channel('speech.googleapis.com', 443))
    
    return requestSoundTranscriptResponse

def sound_transcript_server():
    rospy.init_node('sound_transcript_server')
    while(True):
        try:
            serv = rospy.Service('sound_transcript_server',
                RequestSoundTranscript,
                handle_request_sound_transcript)

            rospy.spin()
            break
        except RuntimeError:
            pass

if __name__ == "__main__":
    sound_transcript_server()

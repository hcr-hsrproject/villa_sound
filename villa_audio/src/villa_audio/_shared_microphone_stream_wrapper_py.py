import numpy as np
from villa_audio._shared_microphone_stream_wrapper_cpp import SharedMicrophoneStreamWrapper

class SharedMicrophoneStream(object):
    def __init__(self):
        self._sms = SharedMicrophoneStreamWrapper()

        # Stream is never closed
        self.closed = False

    """
    Reads from shared memory stream
    and returns bytestring
    Note: Chunk size is ignored
    """
    def read(self, chunk_size):
        # TODO: replace read(False, False) with parameterized version
        return np.array(self._sms.read(False, False), dtype=np.int16).tostring()


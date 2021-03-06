import re
import sys

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
import asyncio
from six.moves import queue
from sensoryBuffer import SensoryBuffer
from log import log

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms


class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1,
            rate=self._rate,
            input=True, 
            frames_per_buffer=self._chunk,
            input_device_index=3, # Run cat /proc/asound/cards to find input for microphone
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return

            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


class TextStream(object):
    def __init__(self):
        self.closed = True

    def __enter__(self):
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        self.closed = True

    def generator(self, responses):
        while not self.closed:
            num_chars_printed = 0

            for response in responses:
                if not response.results:
                    continue

                result = response.results[0]
                if not result.alternatives:
                    continue

                transcript = result.alternatives[0].transcript

                overwrite_chars = ' ' * (num_chars_printed - len(transcript))

                if not result.is_final:
                    num_chars_printed = len(transcript)
                else:
                    yield (transcript + overwrite_chars)


colors = ['red', 'white', 'blue', 'green', 'yellow']
numbers = ['1', '2', '3', '4', '5']
tokens = ['hint token', 'life token', 'hint tokens', 'life tokens']


color_cards = [(color + 'card') for color in colors]
color_cards += [(color + 'cards') for color in colors]
color_number = [(color + number) for color in colors for number in numbers]
determiners = ['your', 'my', 'these', 'those']
units = ['card', 'cards']
noun_phrases = [(det + unit) for det in determiners for unit in units]
common_phrases = ['you have']
filler_words = ['uh', 'um', 'ah']
hardcoded = ['your card at index', 'your cards at indices']

terminologies = (colors + numbers + color_cards + color_number + noun_phrases +
    tokens)

def voice_stream_to_text():
    language_code = 'en-US'  # a BCP-47 language tag

    contexts = types.SpeechContext(phrases = terminologies)

    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
        speech_contexts=[contexts])
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True
        )

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        # listen_print_loop(responses)
        with TextStream() as ts:
            text_generator = ts.generator(responses)
            for text in text_generator:
                yield text

def main(sender):
    for text in voice_stream_to_text():
        log(f"generated {text}")
        sender.send(text)

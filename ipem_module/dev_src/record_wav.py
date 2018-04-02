import pyaudio
import wave
import numpy as np
 
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 22050
CHUNK = 1024
RECORD_SECONDS = 5
 
audio = pyaudio.PyAudio()
 
# start Recording
stream = audio.open(format=FORMAT, channels=CHANNELS,
                    rate=RATE, input=True,
                    frames_per_buffer=CHUNK)
print "recording..."
frames = []
 
for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    np_data = np.fromstring(data, dtype=np.int16).reshape([-1, 2])
    frames.append(np_data)
print "finished recording"
 
 
# stop Recording
stream.stop_stream()
stream.close()
audio.terminate()

all_frames = np.vstack(frames)

print all_frames.shape

waveFile = wave.open('input_L.wav', 'wb')
waveFile.setnchannels(1)
waveFile.setsampwidth(audio.get_sample_size(FORMAT))
waveFile.setframerate(RATE)
waveFile.writeframes(all_frames[:, 0].tostring())
waveFile.close()

waveFile = wave.open('input_R.wav', 'wb')
waveFile.setnchannels(1)
waveFile.setsampwidth(audio.get_sample_size(FORMAT))
waveFile.setframerate(RATE)
waveFile.writeframes(all_frames[:, 1].tostring())
waveFile.close()
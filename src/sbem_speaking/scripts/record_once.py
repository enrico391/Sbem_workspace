import sounddevice as sd
import numpy as np
import wave

SAMPLE_RATE = 44100  # or 48000 if your mic supports it
DURATION = 5  # seconds
OUTPUT_FILE = "output.wav"

print(sd.query_devices())  # optional, lists all devices
sd.default.device = "hw:1,0"  # force ALSA USB mic

print("Recording...")
audio = sd.rec(int(DURATION * SAMPLE_RATE),
               samplerate=SAMPLE_RATE,
               channels=1,
               dtype='int16')
sd.wait()
print("Recording finished.")

with wave.open(OUTPUT_FILE, "wb") as wf:
    wf.setnchannels(1)
    wf.setsampwidth(2)
    wf.setframerate(SAMPLE_RATE)
    wf.writeframes(audio.tobytes())

print(f"Saved to {OUTPUT_FILE}")

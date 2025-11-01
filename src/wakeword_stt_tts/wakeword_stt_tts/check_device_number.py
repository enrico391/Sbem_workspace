import pyaudio

p = pyaudio.PyAudio()

print("Available audio devices:")
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f"Device index {i}: {info['name']} - Input Channels: {info['maxInputChannels']} - Output Channels: {info['maxOutputChannels']} - defualt simple rate {info['defaultSampleRate']}")

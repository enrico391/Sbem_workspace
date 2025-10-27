from whisper_live.client import TranscriptionClient



client = TranscriptionClient(
  "192.168.178.152",
  9090,
  lang="it",
  translate=False,
  model="large-v3",                                      # also support hf_model => `Systran/faster-whisper-small`
  use_vad=False,
  save_output_recording=False,                         # Only used for microphone input, False by Default
  output_recording_filename="./output_recording.wav", # Only used for microphone input
  max_clients=4,
  #max_connection_time=600,
  mute_audio_playback=False,                          # Only used for file input, False by Default
)



client()
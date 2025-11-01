from setuptools import find_packages, setup

package_name = 'wakeword_stt_tts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (('share/' + package_name + '/launch'), ['launch/wakeword_stt_on_device.launch.py']),
        (('share/' + package_name + '/launch'), ['launch/wakeword_stt_on_server.launch.py']),

    ],
    install_requires=[
        'pvporcupine',
        'pvcheetah',
        'pvrecorder',
        'pyaudio',
        'soxr',
        'faster-whisper',
        'openwakeword',
        'gTTS',
        'numpy<1.25.0'
        ],
    zip_safe=True,
    maintainer='morowsl',
    maintainer_email='enrimoro003@gmail.com',
    description='ROS2 package for wake word detection and speech-to-text using Porcupine and Cheetah',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wakeword_stt_whisper_server = wakeword_stt_tts.wakeword_stt_whisper_server:main',
            'tts_server = wakeword_stt_tts.tts_server:main',
            'stt_server = wakeword_stt_tts.stt_server:main',
        ],
    },
)

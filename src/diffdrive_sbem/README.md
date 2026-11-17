# Diffdrive_sbem
Implementation of a differential drive robot using the SBEM platform.

It implement the differential drive controller with a custom hardware interface that connects to the SBEM ESP32 microcontroller over a serial connection.

Furthermore, it implements a battery state broadcaster that publishes the battery state on the `/battery_state` topic. It is not the official ROS2 battery state broadcaster, but a custom implementation that is tailored to the SBEM platform and also because the official one is not yet available for ROS2 Humble.

It uses also a imu broadcaster that publishes the IMU data on the `/imu` topic. 

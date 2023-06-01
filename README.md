### Mosquitto MQTT Publisher Example
The example shows an implementation of MQTT Publisher.
For details please refer to the [documentation](https://click.kaspersky.com/?hl=en-us&link=online_help&pid=kos&version=1.1).

### Prerequisites
Before you begin, ensure that you have met the following requirements:

    You have installed the latest version of KasperskyOS Community Edition
    You have Debian GNU/Linux "Buster" 10 x64

### Usage
To build the example and run the resulting image use the cross-build.sh script.
A static IP address 10.0.2.2 and port 1883 is set for Mosquito broker using
env variables MQTT_BROKER_ADDRESS and MQTT_BROKER_PORT respectively.
You can change the broker's address and port in the root CMakeLists according to your network configuration.

© 2022 AO Kaspersky Lab

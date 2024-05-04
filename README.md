# Title
ROS2 workspace files from ROBO24 robot Pi4 SDCard

## Description
My other repositories are required for the complete project:

[Arduino](https://github.com/mikew123/Arduino) - This code repository has code for each of the microcontrollers which connecct using USB serial to ROS2 topics.

[micropython-stuff-jon](https://github.com/mikew123/micropython-stuff-jon) - This code repository is for the "watch" controller using ESP-now protocol to the Pi4 using an ESP32 board plugged into USB serial. The code for the ESP32 board which sends/receives ESP-now is in the Arduino repository.

This wheeled robot is designed to compete in the DPRG (Dallas Personal Robitics Group) indoor RoboColumbus 
https://www.dprg.org/

The competions it will participate in are 6-can, 4-corner and Quick trip.

There are two versions of each competition; Home and DPRG.<br>
Home is a smaller arena to fit in my office space, the DPRG arean is the competition full size design. For example the home 6-can is 7'x7.5' whereas the DPRG size is 7'x10'

## JSON messages/topics
The ESP-now messages to the watch controller are in JSON string format<br>
The /watch_json topic messages are JSON string formated<br>
The /robo24_json topic messages are JSON string formated<br>
It is recommended that the JSON messages are created using Python Dict constructs which are converted to/from String messages to send over topics and USB serial using json.dumps() and json.loads() routines from the json library (import json)<br>

The message flow watch to ros2 node
- watch->esp-now->esp32_serial->/watch_json->/robo24_json->ros2_node<br>

The message flow ros2 node to watch
- ros2_node->/robo24_json->/watch_json->esp32_serial->esp-now->watch<br>

The message flow ros2 node to node
- ros2_node->/robo24_json->ros2_node<br>


### /watch_json topic JSON messages
Topic messages to the watch are formated as msg_json={"send": {  JSON   }}. Use msg_json_str=json.dumps(msg_json) to covert to a JSON formated string.<br>
Topic messages from the watch are formated as {"revc": {  JSON   }}. Use json.loads() to covert from a JSON formated string.

#### Watch Send JSON messages
Messages from robot to watch Format: msg_json={"send": {  JSON   }}<br>
The { JSON } part of the message is sent to the watch controller over ESP-now<br>
- {"rv" : volts, "ra": amps} This is the main battery Voltage and Current in float format.<br>
- {"nav": {"mode": "mode, "state": "state"}, "t_name": "waypoint"} This is the navigator status in string formats<br>
  Example: {"nav": {"mode": "6-can", "arena": "home"}, "state": "running", "waypoint": "can"}<br>
  
#### Watch Recv JSON messages
Messages from watch to robot Format: msg_json={"recv": {  JSON   }}<br>
The { JSON } part of the message is sent from the watch controller over ESP-now<br>
- {"nav": {"mode": "6-can", "arena": "dprg"}, "state": "init"}<br>
- {"reset": "true"}<br>

### /robo24_json topic JSON messages
Command topic messages {"cmd": { JSON }}<br
Status topic messages {"status": { JSON }}<br>

- {"claw": {"open": pct, "time": msec}}<br>
  Example: {"claw": {"open": 100, "time": 1000}} Claw opens all the way in 1 second





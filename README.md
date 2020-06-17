![Restraining Bolt](https://github.com/bitdog-io/restraining_bolt/raw/pre-release/images/restraining_bolt.png)
==================

*Last Updated: 8 June 2020*

I have a very large rover that lost control one day while executing an automated mission. 
It took me 21 seconds to respond and by that time the rover got very close to causing a disaster. 

Although geo-fencing might be a solution from some run away conditions, 
unfortuantely, that is not always a valid solution. Having a rover off its path, 
but still within a valid geo-fenced area, can still be a dangerous situation. I discovered
I couldn't depend on just the flight controller, geo-fenced areas, or my reaction time for safety. That is why I made this project. Restraining Bolt is designed to make sure your robot doesn't run away.

This software enables Teensy 4.1 microcontrollers to become simple onboard companion computers 
for ArduPilot based rovers. It integrates with ArduPilot via MAVLink protocol over 
serial telemetry to determine if the connected rover is progressing towards waypoints 
while in automated flight. If the software detects that progress isn't being made in 
the direction of the target waypoint, a PWM signal from the Teensy GPIO will enable propulsion system shutdown.

I picked Teensy 4.1 hardware for this project for a variety of reasons. Firstly, its one of the few Arduino
based boards that have enough RAM to use the MAVLink C library without modification. It also provides an
onboard SD card slot. And finalily, it has pretty decent hardware to make sound. I plan on using this last
feature to enable Restraining Bolt to verbally announce important state changes and warnings.

## Getting started
The things you will need:

- Teensy 4.1 board & Micro SD Card
- Learn how to use [Teensy toolchain](https://www.pjrc.com/teensy/tutorial.html)
- Get a PWM controlled [relay like](https://www.amazon.com/dp/B01M3WQZLF/ref=cm_sw_em_r_mt_dp_U_Ni51EbRXS7CVA)
- If you want audio prompts you might also want to get a small audio amp like [this](https://www.sparkfun.com/products/11044)

Copy the contents of /sdcard found in this repo to the SD card you are using for your Teensy. Open the config.ini on the SD card
and make any necessary changes.

## Compilation
I have include the external libraries as a zip under /libraries. You will need to extract the zip file and 
and move the extracted folders to (if you use Windows)  C:\Users\yourusername\Documents\Arduino\libraries.

Note: Most of the libraries I use like Arduino-Log, sdconfigfile, and TaskScheduler can be found as public
libraries in the Arduino IDE. I also use mavlink2 which I downloaded from
[GitHub mavlink/c_library_v2](https://github.com/mavlink/c_library_v2) repo. I modified it a bit to eliminate any
compiler warning that might confuse users.

## Testing
To test the logic in this program I made it easy to use Mission Planner telemetry logs instead of real MAVLink telemetry.
Just load a copy of a recorded mission onto an SD card and change the config.ini to point to it.

## Changing sound prompts
I used [TTSAutomate](https://ttsautomate.com/) to generate the voice prompts used in this program. Install TTSAutomate 
then use it to open english.psv which can be found in this repo. Any newly generated prompts need to be copied to the 
SD card under /sounds.

## Setup
Plug Teensy's serial 1 line into a telemetry port on the flight controller. Make sure to go into Mission Planner and set 
the protocol for the telemetry port on the flight controller as MAVLink 2 with a baud rate of 57600. 

Attach pin 33 to the power system RC relay;
Attach pin 36 to the optional alarm RC relay;
Attach optional amp and speaker to MQSL (left) and MQSR (right) for stereo, or just MQSL for mono. All prompts are generated in mono.

Note: Be very careful, the pinout diagrams provided by pjrc.com show logical pin numbers, not physical. Logical pin 12 is not the 
same as counting to pin 12 on the pcb. Diagram [here](https://www.pjrc.com/wp-content/uploads/2020/05/teensy41_card.png)

## Operation
When Restraining Bolt starts it begins consuming MAVLink 2.0 telemetry messages from the flight controller. It first sends
a PWM signal to an RC relay that will enable power for the rover drivetrain. It also also sends a signal to disable an optional 
alarm. It will detect when the rover is put into AUTO mode and start monitoring the mission. If there is a failure of telemetry
coming from the flight controller, or if the rover swings off course for X seconds, then the software will stop sending PWM signal
to the RC rely thus killing power. It will also send signal to the optional alarm. If the optional amp and speaker are attached,
it will also verbally announce state changes and alarms. When the filght controller is put back to a mode other than auto, it will 
reset and resume good signals to power the rover.

Restraining Bolt also monitors GPS fix status. If a minimum fix status isn't maintained by at least one GPS, Restraining Bolt will 
attempt to pause the mission until at least one GPS is reporting minimum fix status.

I used PWM based RC relays as a form of secondary hardware check. It is very unlikely that a bad microcontroller would still produce a good
PWM signal to the RC relay and power the rover.

## Future
 - I plan on creating a gas engine monitor and control system and include it as an optional feature in ths software.
 - I might also add secondary GPS checking via serial port. 
 - LCD display screen might also be nice.
 - Sending back MAVLink updates from Restraining Bolt to listening base stations might be useful .



  
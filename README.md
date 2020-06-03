![Restraining Bolt](https://github.com/bitdog-io/restraining_bolt/raw/pre-release/images/restraining_bolt.png)
==================

*Last Updated: 2 June 2020*

I have a very large rover that lost control one day while executing an automated mission. 
It took me 21 seconds to respond and by that time the rover got very close to causing a disaster. 

Although geo-fencing might be a solution from some run away conditions, 
unfortuantely, that is not always a valid soltuion. Having a rover off its path, 
but still within a valid geo-fenced area, can still be a dangerous situation. I discovered
I couldn't depend on just the flight controller, geo-fenced areas, or my reaction time for safety. That is why I made this project. Restraining Bolt is designed to make sure your robot doesn't run away.

This softare enables Teensy 4.1 microcontrollers to become simple onboard companion computers 
for ArduPilot based rovers. It integrates with ArduPilot via MAVLink protocol over 
serial telemetry to determine if the connected rover is progressing towards waypoints 
while in automated flight. If the software detects that progress isn't being made in 
the direction of the target waypoint, a PWM signal from the Teensy GPIO will enable propulsion system shutdown.

I picked Teensy 4.1 hardware for this project for a variety of reasons. Firstly, its one of the few Arduino
based boards to have enough ram to use the MALLink C library without modification. It also provides an
onboard SD card slot. And finalily, it has pretty decent hardware to make sound. I plan on using this last
feature to enable Restraining Bolt to make audible warnings about state changes.

## Getting started
The things you will need:

- Teensy 4.1 board & Micro SD Card
- Learn how to use [Teensy toolchain](https://www.pjrc.com/teensy/tutorial.html)
- Get a PWM controlled [relay like](https://www.amazon.com/dp/B01M3WQZLF/ref=cm_sw_em_r_mt_dp_U_Ni51EbRXS7CVA)
- You might also want to get a small audio amp like [this](https://www.sparkfun.com/products/11044)

## Compilation
I have include the external libraries as a zip under /libraries. You will need to extract the zip file and 
and move the extracted folders to (if you use Windows)  C:\Users\{your-username}\Documents\Arduino\libraries.

Note: Most of the libraries I use like Arduino-Log, sdconfigfile, and TaskScheduler can be found as public
libraries in the Arduino IDE. I also use mavlink2 which I cobbled together from
[GitHub mavlink/c_library_v2](https://github.com/mavlink/c_library_v2) repo. I modified it a bit to eliminate any
compiler warning that might confuse users.

## Wiring 

![Restraining Bolt](https://github.com/bitdog-io/restraining_bolt/raw/pre-release/images/teensy41.png)


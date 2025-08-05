This Code is part of an article on SwanRobotics.com: https://swanrobotics.com/esp-now-remote-control-test/

To make it work for your setup, you'll need to update the MAC addresses in the code. The code handles several tasks:

The remote sends a heartbeat every 200 ms. This is the number of milliseconds that the remote has been active. The robot checks for incoming data. If no data comes in, the servo returns to its middle position. This is important when the system is used to control an actual robot.

The joystick value is sent to the robot. The robot reads this value and converts it to a servo position.

The distance measured by the sensor is read and sent back to the remote.

The data from both ESP boards can be monitored via the Serial Monitor in the Arduino IDE. This makes it easy to test the connection in different locations.

Once the code is uploaded, it should be possible to view the correct data.

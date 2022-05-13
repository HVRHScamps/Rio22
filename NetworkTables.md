# List of NT entries for Raspberry Pi to RoboRio communications
Note: Not all of these are implimented in current commit.
## Sensor Table
| Key | Type | Description |
| ----------- | ----------- | ----------- |
| Override | bool | True when RoboRio not in autonomous mode |
| Enable  | bool | True when robot enabled |
| shootEncoder | double | value of the shoot hood encoder |
| driveEncoderL | double | value of the left drivetrain encoder |
| driveEncoderR | double | value of the right drivetrain encoder |
| shootCounter | double | geartooth counter value for shoot wheel speed |
| colorProximity | double| Proximity as determined by color sensor |
| colorSensor | array of doubles | R G B values of color sensor

## Control Table
| Key | Type | Description |
| ----------- | ----------- | ----------- |
deadman | double | Every cycle Pi will update this with a new value, if it doesn't change assume comms fault
stop | bool | rasberry Pi resquests stop of all actuators (soft disable)
driveL | double | requested speed for left drivetrain
driveR | double | requested speed for right drivetrain
beltZ1 | double | requested speed for belt zone 1
beltZ2 | double | requested speed for belt zone 2
beltZ3 | double | requested speed for belt zone 3
shootWhl | double | requested speed for shoot wheel
shootPos | double | requested value for shoot hood jackscrew (only valid values are -1, 0, 1)
climber | double | requested speed for climber motor
climberServo | double | requested angle for climb locker servo
pickupM | double | requested speed for pickup motor
clrPnlM | double | requested speed for color panel motor
clrPnlPNM | double | value for color panel pneumatic solenoid
pickupPNM | double | value for pickup arm pneumatic solenoid
lTensPNM | double | value for lower belt tension pneumatic solenoid
uTensPNM | double | value for upper belt tension pneumatic solenoid
encRst | double | Call for encoder reset 

### encRst behavior
| value | action |
| --- | --- |
0 | no action
1 | reset shoot encoder
2 | reset left drive encoder
3 | reset right drive encoder
4 | reset shoot speed counter
After action is performed, RoboRio sets value back to 0
# Search_and_Rescue_Robot

Alternate algorithm for the robot that does not use the path and tile system. All values are based on sensor input.

### void moveForward ()
- set the motor speed
- check if robot needs to adjust
- if justTurnt = true, then only when the robot moves out to the turn range does it allow the robot to try and turn again (set justTurnt = false)
- if it does then break to allow robot to go into adjust left/right function
- set robot to move forward

### void turnRight()
- use TOF to see if robot is in the turning distance (tile width - robot length)
- If robot is in that range then turn
- Make sure robot is within a desired range of where the robot sitting (100 - 150 away from wall)
- If not adjust to be in that range
- set value to justTurnt = true

### void adjust(char direction) 
FUNCTION: check if robot needs to adjust right or left, set state if needed

CONDITIONS: 
    1. TOF reading not within desired tolerance
    2. actual TOF distance is too close/far to wall (out of range)
    3. IMU angle angle is too small/big

### Note: 
- still currently using back TOF
- Code changes seen in **Arduino_main\Arduino_main.ino**

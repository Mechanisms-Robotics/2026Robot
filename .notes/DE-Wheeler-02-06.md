# DE Session Plan
## Team #8736  The Mechanisms

### Date: 02/05/2026
### Time: 18:00 - 21:00
### Location: Wheeler

## Primary Goals

What are the 1-3 "must-haves" to call tonight a success?
* Goal 1: Tune path following and test longer, faster paths (see Drivetrain, Odometry and Path Following Tuning Procedure for details)
* Goal 2: Test vision with multiple April tags, tweak vision on the field
* Goal 3: Understand how odometry and vision work together in the pose estiimator
* Goal 4 (Optional): Test the new treads

## Session Notes:
- Robot on field at 6:20
- Robot radio on Mech not configured to connect to access point
- Missing bumpers
- Recognition of April Tag changed pose which flipped field for field-relative drive
  - Fix for field-relative drive to orient on gyro instead of robotPose already done on Wheeler branch on Mr. Odom's laptop
  - Further work needed..."reset gyro" button doesn't work anymore
- Robot beaches over bump if driving straight over it (note Mech is on raised swerve, comp bot is even lower)
  - driving over at 45 degree skew worked better
- Pathing
  - Rotational path to turn 180 degrees
    - It stopped 5 degrees short. But PoseEstimator knows it is 5 degrees short!
    - will accept this for now, but need to work on more tuning of kP...maybe need an end-state controller?
  - Linear drive of 1m
    - PoseEstimate was 3cm over
    - Kicks to left a bit
    - Worked on tuning kP
      - doubled it over and over until we saw the bot lurching repetitively along the path, then backed off
  - Arc path
    - looked pretty solid
- Vision testing
  - more precise when one of the two hub tags was hidden
  - tested teleporting (picking up robot and moving it)
    - then pointed at april tag -- advantage scope showed the robot then correct the pose based on the april tag
    - we don't understand why it wasn't behaving this way in the shop 1 week ago
- ROBOT CODE CRASHED @ 7:45pm
- tested some longer, faster paths
- 


## ACTIONS:
- Configure robot radio on Mech and ensure connectivity to access point
- Think about simple tools to help navigate the bump (maybe skis that Nolan highlighted from RI3D?, other ideas?)
- Continue tuning Kp for Path Turning
- Fix zeroGyro button
- Continue tuning Kp for Path driving
- Make sure packing list is complete (missing bumpers)
- Figure out how to log properly so we can catch and debug robot crashes

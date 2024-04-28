# FTC Programming
 
This repository contains the code written by me and my colleagues from the programming department of FTC team Ro2D2. 

The code is structured in two main parts , the autonomy code and the drive code.

**Autonomy code** ( Found in the files of the directory DLS-main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/OGCode/Autonomii)

Our team managed to implement the most efficient autonomous period in the world of the 2022-2023 FTC season Power play (https://www.youtube.com/watch?v=IIK3K2E9l90&t=1797s). It can be seen in the Qualification match 5, our team 17962 being the one moving from the blue alliance. We managed to place 10 cones out of the 11 possible during the autonomous period.

To acheive this we used the RoadRunner library to implement the autonomous pathing of the robot albeit with some modifications:
- We decided to remove the correction time of the library as that would have meant we would go over the 30 seconds allocated for the autonomous period
- We also decided to go for the internal PID loop for the motor velocities which was heavily criticised by the FTC community and now it is depracated. However, we found that it offered more accuracy than Feedforward algorithms.

For the autonomous movements of the systems, we wrote our own finite state machine code which helped us in having more control. This also helped us achieve a small loop time which enhanced the responsiveness of the systems.

**Drive code**

When the robot was controlled by the driver we used the same custom finite state machines that are used for the autonomous parts. We also used a simple PID function for the acutation of linear slides and other 
systems that needed to arrive at an exact position. The hardest part of the drive code was coordinating them in such way that they would behave as the driver wanted.

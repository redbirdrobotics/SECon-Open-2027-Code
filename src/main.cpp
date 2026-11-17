/*
In this comment block, the psudocode, variables, and functions for the program is layed out.
NOTE: This was a very early version of the pseudocode and will have multiple errors and missing information.
However, this will be a good starting point for the program and will be updated as the project progresses.

  Parts:
    1. Servo    (3)
    2. DC Motor (4)
    3. LiDAR    (1)
    4. Encoder  (2 if necessary)
    5. IMU      (1)

  Part Variables (ideally):
    1. Servo: servoPin, servoAngle
    2. DC Motor: motorPin, motorSpeed
    3. LiDAR: lidarPin, lidarDistance
    4. Encoder: encoderPin, encoderCount

  Functions that the devices will perform:
    1. Servo functions: raise the claw arm for the tire dropoff
    2. DC Motor functions: Control the drivetrain speed and turning of the car
    3. LiDAR functions: obtain the position of the car in relation to the car's reference frame
    4. Encoder functions: make a closed loop system for the drivetrain of the car and maintain the odometry  (NOTE: no longer true but may change depending on the situation we may find ourselves in)
    5. IMU: obtain multiple values such as the orientation of the drivetrain in relation to the true coordinate system to create the Kalman filter

  Phases of the code:
    Phase 1: detect the start flag
              start the LiDAR communication and wait for the start flag to be detected
              have an infinite loop that checks for the start flag. once detected, break the loop and move to the next phase

    Phase 2: Read the apriltag and determine the pit stop location
              enable data reception from the april tag reader
              once data is taken, disable data reception from the april tag reader
              process the data and determine the pit stop location
              end this task

    phase 3: Make a loop and then go to the drop off location
              Go fowards on the track and use the lidar to detect the obstacles on the course.
              Using control theory, control the motors to stay on the track and avoid obstacles.
              Once the straight path is over, a curve function needs to be made
              The forward is called again because its a loop.

    phase 4: Drop off the tire at the pit stop location
              Once the straight path is over, the car proceeds to the pit stop.
              the april scanner is enabled and the pit stop location is read again to ensure that the car is at the correct location
              a timer is intialized on the on the microcontroller to count to 2 sec
              once the 2 sec is up the car is allowed to move
              (not sure if allowed) while idle, the servo is raised to drop off the tire and lowered back down to its original position

    phase 5: Pick up the tire and continue the course
              One servo will lower and raise the arms and the other servo will open and close the claw to pick up the tire
              The vertical servo will lower, the horizontal servo will open, and the vertical servo will raise

    phase 6: Make another loop and then go to the finish line
              It's a rinse and repeat. Did you really think I was going to be unique about this?

  Functions required for this project:
    Straight: move the car forward on the straight path of the course
                - If an obstacle is detected within a tolerable range, dodge the obstacle
                - The Lidar will work with the IMU and the encoders to keep track of position
    Turn: make a turn on the course
                - We create a target x-value with the same Y-value that the straight function ends at
                - The controller will essentially create the best path to follow to go around the corner 
    Start Timer
                -There 2 timers. One is a kill switch in case the run takes too long and the other is for the mandatory 2 second pit stop
    Read Tag
    Read LiDAR
    Drop Off Tire

  Important variables not associated to the parts:
    1. Start Flag: a boolean that is set true when the start flag is detected
    2. Pit Stop Location: an integer (probably) that stores the location of the pit stop
    3. Time: an integer that stores the time elapsed since the start of the timer
    4. Lap Count: an integer that stores the number of laps completed by the car

NOTE: these pins are not finalized since there isnt even a finalized schematic and not much of a physical circuit either.


bool timerDone        // Flag to indicate if the timer has completed
bool startFlag        // Flag to indicate if the start flag is detected
int  pitStopLocation  // Variable to store the pit stop location
int  lapsCompleted    // Variable to store the number of laps completed
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "zsl/zsl.h"
#include "zsl/matrices.h"
#include "CoordProcess.hpp"
#include "io_core.hpp"
#include "const.hpp"

void controlMotors(int speed, int direction) {
    /*
    So this is where the first control will go.
    The logic here is that there is a target speed and target direction 
    Using control theory and linear algebra, we will optimize the motion of the motors
    */
}
void targetposition(int x, int y) {
    /*
    This is where the second control will go (yes, there will be at least 2 controls).
    The logic here is that there is a target position that the robot must achieve and the robot will keep moving to this position
    However, due to the obstacles on the course, the robot will have to adjust its path to reach the target position
    There may be a third control in here which is meant for the dodging of the obstacles on the course
    This will be determined later on in the project since we only need it to make laps for now
    */
}
int main() {

    stdio_init_all();
    while(!stdio_usb_connected()){
        sleep_ms(100);
    }
    
    io_manager.init();
    io_manager.run();

    while (true) {
        LidarData& data = io_manager.theta_bufs[90];
        mutex_enter_blocking(&data.mtx);
        float r = data.r_data[0];
        mutex_exit(&data.mtx);
        sleep_ms(100);
    }

    io_manager.signal_to_end();
}




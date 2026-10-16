# SECon Open 2027 Robot

This repository contains the C++ source code for the robot.

## Git "best" practice

Be sure to have some prerequisite knowledge of git before contributing, as this is not a git tutorial 

### Branches
Before branching from **dev** branch and pushing changes to **origin/dev**, you should ***always*** pull from **origin/dev** to make sure you have the most update to date changes.

When creating a feature, modifiying docs, refactoring, or anything, please make a branch for it and test it. After it works, merge the branch back into dev branch.

###### ** This is subject to change **
Git Branch Workflow
* main/
	* dev/
		* feat/<>
		* fix/<>
		* docs/<>
		* test/<>
		* refactor/<>
		* chore/<>

### Commits

When writing commit messages:
* commit often 
* prefix the message with what changed
* make sure the commit message fairly detailed

For example, if you are adding a feat use "feat: changes" instead of "vague changes"

Consult additional resources below for more commit message best practice

###### ** This is subject to change **
#### List of common commit prefixes
* feat - for features
* fix - fixed a bug or error
* docs - changed or rewrote documentation
* refactor - refactored a part of the code or the structure of the code
* chore - a simple boring change like fixing formatting
* merge - resolved a merge conflict

### Useful resources
https://www.conventionalcommits.org/en/v1.0.0/

https://git-scm.com/cheat-sheet

https://docs.github.com/en/get-started/using-git

## Libraries
* Servo - https://github.com/markushi/pico-servo
* DC Motors - use Pico SDK
* Encoders - https://github.com/GitJer/Some_RPI-Pico_stuff/tree/main/Rotary_encoder
* Lidar - https://github.com/Slamtec/rplidar_sdk
* IMU - https://github.com/MaikSteiger/rpi-pico-mpu6050 or Pico SDK??

## Custom Libraries
* Motor + Encoder Combo
* Lidar?????
* PID (Might already exist)
* Matrix (Might already exist)
* April tag reader (Via Connor)

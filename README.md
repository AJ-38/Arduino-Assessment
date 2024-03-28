# Arduino-Assessment

Aditya Joshi, C1020235 Assessment

In this repository, I have attached my code for the Zumo robot, named AssessmentNewest.ino. In order for this program to run and work with the robot, if it has not already been installed, a library under the names Vector.h and StackArray.h must be installed. Vector.h can be installed from the inbuilt Arduino library from Peter Polidro, and StackArray.h can be installed directly from the web. The StackArray.h will be found on Github and will need to be zipped. Within Arudino,  you will need to go to the sketch section and click include library. Then it will provide an option in the next drop-down window saying to add .ZIP library. The StackArray.h library can then be included. I have included StackArray.h as it shows my trials and tribulations with this project. I had attempted to use it, but decided to use the Vector.h library for my return-to-home attempt as it is simpler and easier to follow. The code should count how long it has been since a turn has been made until the next turn, and then stop the timer. As it is in a loop, the timer does not need to be continuously restarted and can simply be stopped afterwards. Timer.h library will also need to be installed, which can be found within the Arduino library. by Stefan Staub. This code, with the timer should then reverse the actions that have been taken using the vector that has been created, using the time elapsed and the direction to turn in to ensure that the exact opposite action is taken from the previous actions.

The Zumo robot will need to be connected then, and then the code should be compiled and uploaded onto it. Once uploaded, the Zumo can be disconnected and placed on the maze. The button A will need to be pressed to calibrate the front left and right line sensors; ideally, the Zumo should be placed half over the white area and half over a line so it can calibrate the differences between the black and white on the maze as it rotates. It will perform two 180 degree turns, and then it will return to its original positioning ( depending on the sensors, it may be slightly angled differently after calibration).

Then, the zumo robot can be positioned at the desired starting position on the maze, and the button A on the zumo will need to be pressed again so that the robot can begin navigating the maze. I had created functions for all of the movement so that it could go forward, backward, left, and right. The zumo will navigate by constantly checking if the sensor is greater than the QTR threshold value; if it is greater than this value, it means one or both sensors have detected a black line. It will then either turn to the left or right depending on which sensor detected the line, and then it will continue moving forward. If both sensors detect a line, the Zumo will reverse and turn to the right three times to complete a 360-degree turn and continue driving. This code is to ensure the Zumo can get out of dead ends. I have also included proximity sensor coding, I have set the objectValue (value for a house or object) at 12, as it is a combination of the readings of both sensors when an object is roughly 15cm in front of the Zumo. The zumo will then bleep and reverse away from the object. I had included some code to do a random direction turn to get around the said object; however, in the specification, we were informed the objects will be placed in dead ends and will not require moving around but will require reversing and turning away from. This code will be commented out.

I had set the value for QTR Threshold (Black Line) by placing my Zumo on the maze while connected to Arduino by wire through the computer, and then had the sensors print out what they were detecting. Doing this allowed me to fine-tune the value so that there is minimal over-running on lines, as we do not want the robot to be driving through people's walls.

The return to home function should in theory work, and all the code is presented; however, under testing, it has not performed as intended and does not return to its starting position.

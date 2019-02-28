#include "robot-config.h"


vex::competition Competition;
/*
*
*
* Global variable that determines that robot's state when connected to a competition switch
*
*
*/
void userControl( void );
/*
*
*
* The main user control method, implements all functions
*
*
*/
void auton( void );
/*
*
*
* The main autonomous method, implements all functions
*
*
*/
void driveFunction(); 
/*
*
*
* Function with the drive code
*
*
*/
void intakeFunction();
/*
*
*
* Function with the intake code 
*
*
*/
void shooterFunction();
/*
*
*
* Function with the shooter code 
*
*
*/
void autonSelector();
/*
*
*
* Function that allows the user to select the auton when the program runs
*
*/
void preAuton();
/*
*
*
* Function that helps set up the autonomous mode, helps cancel out all threads
*
*
*/
void blueFront();
/*
*
*
* Function that contains the code for the blue front auton
*
*
*/
void blueBack(); 
/*
*
*
* Function that contains the code for the blue back auton 
*
*
*/
void redFront(); 
/*
*
*
* Function that contains the code for the red front auton
*
*
*/
void redBack();
/*
*
*
* Function that contains the code for the red back auton
*
*
*/
void progSkills();
/*
*
*
* Function that contains the code for programming skills
*
*
*/
int autonSelect = 0;
/*
*
*
* Global variable that is changed based off of which auton is selected
*
*
*/
bool platform;
/*
*
*
* Global variable that is changed based off of whether the platform option in the auton selecter function is chosen or not
*
*
*/
vex::thread intakeThread; 
/*
*
*
* The thread that runs the intake function
*
*
*/
vex::thread shooterThread; 
/*
*
*
* The thread that runs the shooter function
*
*
*/
double SHOOTER_ENCODER_VALUE = 1800;
/*
*
*
* Global variable that determines how many ticks the shooter motors must cycle for the catapult to shoot and reset
*
*
*/

struct PID {
    double error; 
    double kP; 
    double kI;
    double kD; 
    double derivative; 
    double prev_error;
    double integral; 
    double integralLimit;
    double speed;
};
/*
*
*
* Data structure that contains all of the variables relevant to PID, makes it easy to run multiple pid loops
*
*
*/
double pid(PID& pid);
/*
*
*
* function that takes in a PID structure and runs one cycle, returns a resulting speed
*
*
*/
void leds();
/*
*
*
* function that controls the led lights
*
*
*/
void driveTowardsTheFlags();
/*
*
*
* function that drives towards the flags
*
*
*/
void shooterAutonomous();
/*
*
*
* autonomous function for the shooter
*
*
*/
//void pidDrive(double degrees, int dir);
void pidTurn(double degrees);
void speedDrive(double degrees, int dir, int speed);
void turn(double degrees);
void shooterAutonomousCycle(int stage);
void pidDrive(double degrees, int dir);

int main() {
    //setting global variables to avoid possible null errors
    vex::thread LEDTHREAD = vex::thread(leds);
    Competition.autonomous( auton );
    Competition.drivercontrol( userControl );
    autonSelect = 1;
    platform = true; 
    //running the auton selector
    autonSelector();
    //setting the call back functions of the competition

    //sleeping
    while( true ){
        vex::task::sleep( 10 );
    }
    
}

void driveFunction(){
      //if the up arrow button is pressed, move forward slowly
      if( Controller.ButtonUp.pressing() ){
        LD1.spin( vex::directionType::fwd, 30, vex::velocityUnits::pct );
        LD2.spin( vex::directionType::fwd, 30, vex::velocityUnits::pct );
        RD1.spin( vex::directionType::fwd, 30, vex::velocityUnits::pct );
        RD2.spin( vex::directionType::fwd, 30, vex::velocityUnits::pct );
    }
    //if the down arrow button is pressed, move back slowly
    else if( Controller.ButtonDown.pressing() ){
        LD1.spin( vex::directionType::fwd, -30, vex::velocityUnits::pct );
        LD2.spin( vex::directionType::fwd, -30, vex::velocityUnits::pct );
        RD1.spin( vex::directionType::fwd, -30, vex::velocityUnits::pct );
        RD2.spin( vex::directionType::fwd, -30, vex::velocityUnits::pct );
    }
    //if the right arrow button is pressed, turn right slowly
    else if( Controller.ButtonRight.pressing() ){
        LD1.spin( vex::directionType::fwd, -20, vex::velocityUnits::pct );
        LD2.spin( vex::directionType::fwd, -20, vex::velocityUnits::pct );
        RD1.spin( vex::directionType::fwd, 20, vex::velocityUnits::pct );
        RD2.spin( vex::directionType::fwd, 20, vex::velocityUnits::pct );
    }
    //if the left arrow button is pressed, turn left slowly
    else if( Controller.ButtonLeft.pressing() ){
        LD1.spin( vex::directionType::fwd, 20, vex::velocityUnits::pct );
        LD2.spin( vex::directionType::fwd, 20, vex::velocityUnits::pct );
        RD1.spin( vex::directionType::fwd, -20, vex::velocityUnits::pct );
        RD2.spin( vex::directionType::fwd, -20, vex::velocityUnits::pct );
    }
    else{
        //if nothing else, set the motor speeds to that of the joysticks
        LD1.spin( vex::directionType::fwd, Controller.Axis3.value() - Controller.Axis1.value(), vex::velocityUnits::pct );
        LD2.spin( vex::directionType::fwd, Controller.Axis3.value() - Controller.Axis1.value(), vex::velocityUnits::pct );
        RD1.spin( vex::directionType::fwd, Controller.Axis3.value() + Controller.Axis1.value(), vex::velocityUnits::pct );
        RD2.spin( vex::directionType::fwd, Controller.Axis3.value() + Controller.Axis1.value(), vex::velocityUnits::pct );
    }  
    
}

void intakeFunction(){
    while( true ){
       if( Controller.ButtonR1.pressing() ){
           //intaking balls
           I1.spin( vex::directionType::fwd, 100, vex::velocityUnits::pct );
           I2.spin( vex::directionType::fwd, 100, vex::velocityUnits::pct );
       }    
       else if( Controller.ButtonL1.pressing() ){
           //flipping caps
           I1.spin( vex::directionType::rev, 100, vex::velocityUnits::pct );
           I2.spin( vex::directionType::rev, 100, vex::velocityUnits::pct );
       }
       else{
           //stop the motors
           I1.stop( vex::brakeType::coast );
           I2.stop( vex::brakeType::coast );
       }
    }
}

void shooterFunction(){
    while( true ){
        if( Controller.ButtonA.pressing() ){
            //free control of the shooter
            S1.spin( vex::directionType::fwd, 50, vex::velocityUnits::pct );
            S2.spin( vex::directionType::fwd, 50, vex::velocityUnits::pct );
        }
        else if( Controller.ButtonR2.pressing() ){
            //making a full rotation (shooting the ball and then going to the loading position)
            double initial = S1.rotation( vex::rotationUnits::deg );
            while( S1.rotation( vex::rotationUnits::deg ) < initial + SHOOTER_ENCODER_VALUE ){
                S1.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
                S2.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
            }
        }
        else{
            //stop the motors
            S1.stop();
            S2.stop();
        }
    }    
}

void userControl( void ){
    //setting the threads to their functions
    intakeThread = vex::thread( intakeFunction );
    shooterThread = vex::thread( shooterFunction );
    //running the drive on the main thread
    while( true ){
        driveFunction();
    }
    
}

void preAuton(){
    //interrupting all threads so that the motors can be used in auton functions
    intakeThread.interrupt();
    shooterThread.interrupt();
}

void auton( void ){
    //run the preAuton() function to prepare for the autonomous period
    preAuton();
    //based off of the autonSelect value, run the correct function
    switch( autonSelect ){
        case 1: 
            blueFront();
            break;  
        case 2: 
            blueBack();
            break; 
        case 3: 
            redBack();
            break; 
        case 4: 
            redFront();
            break;
        case 5: 
            progSkills();
            break;
    }
    
}

void autonSelector(){
  //local variable that is used to stop the while loop when the cortex is touched
  bool pressed = false;
  while( !pressed ){ 
        //drawing the rectangles on the screen for the user to select
        Brain.Screen.drawRectangle( 0, 0, 167, 125, vex::color::red );
        Brain.Screen.drawRectangle( 0, 125, 167, 250, vex::color::red );
        Brain.Screen.drawRectangle( 167, 0, 334, 125, vex::color::blue );
        Brain.Screen.drawRectangle( 167, 125, 334, 250, vex::color::blue );
        Brain.Screen.drawRectangle( 334, 0, 500, 250, vex::color::purple );
       
        //writing the options on the screen in each box
        Brain.Screen.setPenColor( vex::color::white);
        Brain.Screen.printAt( 50, 60, "Red Close" );
        Brain.Screen.printAt( 50, 185, "Red Far" );
        Brain.Screen.printAt( 210, 60, "Blue Close" );
        Brain.Screen.printAt( 210, 185, "Blue Far" );
        Brain.Screen.printAt( 350, 125, "Prog Skills" );
        //setting the autonSelect variable to a value based off where the cortex was touched
        if( ( Brain.Screen.pressing() ) && ( Brain.Screen.xPosition() >= 0 && Brain.Screen.xPosition() < 167 ) && ( Brain.Screen.yPosition() >= 0 && Brain.Screen.yPosition() < 125 ) ){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt( 50, 50, "Red Close Auton Selected!" );
            autonSelect = 4;
            pressed = true;
        }
        else if( ( Brain.Screen.pressing() ) && ( Brain.Screen.xPosition() >= 167  && Brain.Screen.xPosition() < 334 ) && ( Brain.Screen.yPosition() >= 0 && Brain.Screen.yPosition() < 125 ) ){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt( 50, 50, "Blue Close Auton Selected!" );
            autonSelect = 1;
            pressed = true;
        }
        else if( ( Brain.Screen.pressing() ) && ( Brain.Screen.xPosition() >= 167  && Brain.Screen.xPosition() < 334) && ( Brain.Screen.yPosition() >= 125 && Brain.Screen.yPosition() < 250 ) ){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt( 50, 50, "Blue Far Auton Selected!" );
            autonSelect = 2;
            pressed = true;
        }
        else if( ( Brain.Screen.pressing() ) && ( Brain.Screen.xPosition() >= 0  && Brain.Screen.xPosition() < 167 ) && ( Brain.Screen.yPosition() >= 125 && Brain.Screen.yPosition() < 250 ) ){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt( 50, 50, "Red Far Auton Selected!" );
            autonSelect = 3;
            pressed = true;
        }
        else if( ( Brain.Screen.pressing() ) && ( Brain.Screen.xPosition() >= 334 && Brain.Screen.xPosition() < 500 ) && ( Brain.Screen.yPosition() >= 0 && Brain.Screen.yPosition() < 250 ) ){
          Brain.Screen.clearScreen();
          Brain.Screen.printAt( 50, 50, "Programming Skills Selected!" );
          autonSelect = 5; 
          pressed = true;
        }
        
        
        vex::task::sleep(200); 
    }
    
    if( autonSelect != 5 ){
        //if the auton selected was not programming skills, then ask if platform is wanted
        pressed = false; 
        while(!pressed ){
            //drawing the rectangles on the screen for the user to select
            Brain.Screen.drawRectangle( 0, 0, 250, 250, vex::color::green );
            Brain.Screen.drawRectangle( 250, 0, 500, 250, vex::color::orange );

            //writing the options in each of the corresponding rectangles 
            Brain.Screen.setPenColor( vex::color::white );
            Brain.Screen.printAt( 100, 50, "Do you want to go on the platform?" );
            Brain.Screen.printAt( 80, 110, "YES" );
            Brain.Screen.printAt( 350, 110, "NO" );
            
            //changing the platform variable based off where the cortex was tapped
            if( ( Brain.Screen.pressing() ) && ( Brain.Screen.xPosition() >= 0 && Brain.Screen.xPosition() < 250 ) ){
                Brain.Screen.clearScreen();
                Brain.Screen.printAt( 50, 100, "PLATFORM" );
                pressed = true;
                platform = true;
            }
            else if( ( Brain.Screen.pressing() ) && ( Brain.Screen.xPosition() >= 250 && Brain.Screen.xPosition() < 500 ) ){
                Brain.Screen.clearScreen();
                Brain.Screen.printAt(50, 100, "NO PLATFORM");
                pressed = true;
                platform = false;
            }
            vex::task::sleep( 200 );
        }

    }
    //print something on the screen based off what auton was selected
    switch( autonSelect ){
        case 1: 
            Brain.Screen.printAt( 50, 50, "Blue Close Auton Selected!" );
            break;  
        case 2: 
            Brain.Screen.printAt( 50, 50, "Blue Far Auton Selected!" );
            break; 
        case 3: 
            Brain.Screen.printAt( 50, 50, "Red Far Auton Selected!" );
            break; 
        case 4: 
            Brain.Screen.printAt( 50, 50, "Red Close Auton Selected!" );
            break;
        case 5: 
            Brain.Screen.printAt( 50, 50, "Programming Skills Selected!" );
            break;
    }
    
    
}

void shooterAutonomousCycle(int stage){
    int h = 1800;
    if(stage == 1){
        h = 1860;
    }
    else{
        h = 1740;
    }
    
    double initial = S1.rotation( vex::rotationUnits::deg );
    while( S1.rotation( vex::rotationUnits::deg ) < initial + h ){
        S1.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
        S2.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
    }
    S1.stop();
    S2.stop();
    
}
void redFront(){

    
    pidDrive(1100, 1);
    vex::task::sleep(200);
    pidDrive(-1100, 1);
    vex::task::sleep(200);
    pidTurn(-305);
    vex::task::sleep(200);
    pidDrive(-700, 1);
    vex::task::sleep(200);
    shooterAutonomousCycle(2);
    
    
    /*speedDrive(1100, 1, 60);
    vex::task::sleep(200);
    I1.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
    vex::task::sleep(500);
    I1.stop();
    speedDrive(-1300, 1, 60);
    vex::task::sleep(400);
    speedDrive(130, 1, 60);
    vex::task::sleep(200);
       
    turn(-243);
    //shoot the balls
    vex::task::sleep(400);
    speedDrive(-218, 1, 60);
    vex::task::sleep(400);
    I1.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    vex::task::sleep(500);
    I1.stop();
    shooterAutonomous();
    vex::task::sleep(400);
    speedDrive(-250, 0, 70);
    vex::task::sleep(200);
    turn(30);
    vex::task::sleep(200);
    speedDrive(-350, 0, 80);
    vex::task::sleep(200);
    turn(30);
    vex::task::sleep(200);
    speedDrive(600, -1, 80);
    vex::task::sleep(200);
    
    */
}

void redBack(){
    speedDrive(1200, 1, 40);
    vex::task::sleep(200);
    speedDrive(-10, 1, 40);
    I1.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct); 
    I2.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
    vex::task::sleep(500);
    I1.stop();
    I2.stop();
    if(platform){
        vex::task::sleep(200);
        turn(-240);
        vex::task::sleep(200);
        speedDrive(-150, 0, 10);
        vex::task::sleep(200);
        speedDrive(300, 0, 10);
        speedDrive(-1500, 0, 95);
    }
}

void blueFront(){
    speedDrive(1100, 1, 80);
    vex::task::sleep(200);
    speedDrive(-1250, 1, 80);
    vex::task::sleep(400);
    speedDrive(200, 1, 40);
    vex::task::sleep(200);

    if(platform){
        
        turn(250);
        //shoot the balls
        vex::task::sleep(400);
        shooterAutonomous();
        vex::task::sleep(200);
        speedDrive(650, 0, 40);
        vex::task::sleep(200);
        turn(240);
        vex::task::sleep(200);
        speedDrive(-2150, 0, 95);
    }
    else{
        turn(230);
        vex::task::sleep(200);
        speedDrive(-700, 1, 80);
        vex::task::sleep(400);
        turn(17);
        vex::task::sleep(400);
        speedDrive(-800, 1, 80);
        speedDrive(1300, 1, 40);
        vex::task::sleep(200);
        shooterAutonomous();
    }

}

void blueBack(){
    speedDrive(1200, 1, 40);
    vex::task::sleep(200);
    speedDrive(-20, 1, 40);
    I1.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct); 
    I2.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
    vex::task::sleep(500);
    I1.stop();
    I2.stop();
    if(platform){
        vex::task::sleep(200);
        turn(240);
        vex::task::sleep(200);
        speedDrive(-150, 0, 10);
        vex::task::sleep(200);
        speedDrive(300, 0, 10);
        speedDrive(-1500, 0, 95);
    }
}

void progSkills(){

}

void turn(double degrees){
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation();
    int speed = 40;
    if(degrees < 0){
        speed = -speed;
    }
    int cur = LD1.rotation(vex::rotationUnits::deg);
    while(abs(cur) < abs(degrees)){
        cur = LD1.rotation(vex::rotationUnits::deg);
        LD1.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        LD2.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        RD1.spin(vex::directionType::fwd, -speed, vex::velocityUnits::pct);
        RD2.spin(vex::directionType::fwd, -speed, vex::velocityUnits::pct);
            
    }
    
    LD1.stop(); 
    RD1.stop(); 
    RD2.stop(); 
    LD2.stop();
    I1.stop();
    I2.stop();
    
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation(); 
    
}

void speedDrive(double degrees, int dir, int speed){
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation();
    
    int cur = LD1.rotation(vex::rotationUnits::deg);
    if(degrees < 0){
        speed = -speed;
    }
    while(abs(cur) < abs(degrees)){
        LD1.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        LD2.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        RD1.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        RD2.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        cur = LD1.rotation(vex::rotationUnits::deg);
        I1.spin(vex::directionType::fwd, dir * 60, vex::velocityUnits::pct);
        I2.spin(vex::directionType::fwd, dir * 60, vex::velocityUnits::pct);
    }
    
    LD1.stop(); 
    RD1.stop(); 
    RD2.stop(); 
    LD2.stop();
    I1.stop();
    I2.stop();
    
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation();   
}

void shooterAutonomous(){
    double initial = S1.rotation( vex::rotationUnits::deg );
    while( S1.rotation( vex::rotationUnits::deg ) < initial + SHOOTER_ENCODER_VALUE ){
        S1.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
        S2.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
    }
    S1.stop();
    S2.stop();
}

double pid(PID& pid){
    pid.derivative = pid.error - pid.prev_error; 
    pid.prev_error = pid.error;
    
    if(abs(pid.error) < pid.integralLimit && (pid.error != 0)){
        pid.integral += pid.error;
    }
    else{
        pid.integral = 0;
    }
    pid.speed = (pid.kP * pid.error) + (pid.integral * pid.kI) + (pid.derivative * pid.kD);
    return pid.speed;
}

void pidDriveVision(){
    
}

void pidTurn(double degrees){
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation();
    
    //setting up the main PID Loop for driving straight
    PID mainPid;  
    mainPid.error = degrees - LD1.rotation(vex::rotationUnits::deg);
    mainPid.kP = 0.149;
    mainPid.kI = 0.0001; 
    mainPid.kD = 0; 
    mainPid.derivative = 0;
    mainPid.prev_error = mainPid.error; 
    mainPid.integral = 0; 
    mainPid.integralLimit = 10; 
    mainPid.speed = 0;
    double leftSpeed = mainPid.speed; 
    double rightSpeed = mainPid.speed; 
    
    while(mainPid.error != 0){
       mainPid.error = degrees - LD1.rotation(vex::rotationUnits::deg);
       if(abs(mainPid.error) < 0.1){
           mainPid.error = 0;
       }
       if(mainPid.error > 800){
           if(degrees > 0 ){
                leftSpeed += 20; 
                if(leftSpeed >= 85){
                    leftSpeed = 85;
                }
           }
           else{
                leftSpeed -= 20; 
                if(leftSpeed <= -85){
                    leftSpeed = -85;
                }    
            }

       }
       else{
           leftSpeed = pid(mainPid);
       }
       rightSpeed = -leftSpeed;

       LD1.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
       LD2.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
       RD1.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
       RD2.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
    }
    
    LD1.stop(); 
    RD1.stop(); 
    RD2.stop(); 
    LD2.stop();

    
    
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation();
    
}


void pidDrive(double degrees, int dir){
    
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation();
    
    //setting up the main PID Loop for driving straight
    PID mainPid;  
    mainPid.error = degrees - LD1.rotation(vex::rotationUnits::deg);
    mainPid.kP = 0.155;
    mainPid.kI = 0.0001; 
    mainPid.kD = 0; 
    mainPid.derivative = 0;
    mainPid.prev_error = mainPid.error; 
    mainPid.integral = 0; 
    mainPid.integralLimit = 10; 
    mainPid.speed = 0;
    double leftSpeed = mainPid.speed; 
    double rightSpeed = mainPid.speed; 
    
    //setting up the drifting PID Loop for correcting drift 
    
    PID driftPid; 
    driftPid.error = LD1.rotation(vex::rotationUnits::deg) - RD1.rotation(vex::rotationUnits::deg);
    driftPid.kP = 0.1;
    driftPid.kI = 00; 
    mainPid.kD = 0; 
    driftPid.derivative = 0;
    driftPid.prev_error = driftPid.error; 
    driftPid.integral = 0; 
    driftPid.integralLimit = 0; 
    driftPid.speed = 0;
    double difference = 0;
    while(mainPid.error != 0){
       mainPid.error = degrees - LD1.rotation(vex::rotationUnits::deg);
       if(abs(mainPid.error) < 0.1){
           mainPid.error = 0;
       }
       if(abs(mainPid.error) > 500){
           if(degrees > 0 ){
                leftSpeed += 20; 
                if(leftSpeed >= 80){
                    leftSpeed = 80;
                }
           }
           else{
                leftSpeed -= 20; 
                if(leftSpeed <= -80){
                    leftSpeed = -80;
                }    
            }

       }
       else{
           leftSpeed = pid(mainPid);
       }
       
       rightSpeed = leftSpeed;
        
       if(degrees > 0){
           driftPid.error = LD1.rotation(vex::rotationUnits::deg) - RD1.rotation(vex::rotationUnits::deg);
           if(driftPid.error != 0){
               difference = abs(pid(driftPid));
               if(degrees < 0){
                   leftSpeed -= difference; 
                   rightSpeed += difference; 
               }
               else{
                   leftSpeed += difference; 
                   rightSpeed -= difference;
               }
           }
        }
        
       LD1.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
       LD2.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
       RD1.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
       RD2.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
       if(dir == 1){
           I1.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
           I2.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
       }
       else if(dir == -1){
           I1.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
           I2.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);  
       }
    }
    
    LD1.stop(); 
    RD1.stop(); 
    RD2.stop(); 
    LD2.stop();
    I1.stop();
    I2.stop();
    
    
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation();
    
    
}


void driveTowardsTheFlags(){
    //this is assuming that the robot is turned towards the flags
    Vision.takeSnapshot(BLUE_FLAG);
    Vision.setLedColor(255, 0, 0);
    Brain.Screen.printAt(50, 150, "%d", Vision.largestObject.centerX);
    Brain.Screen.setFillColor(vex::color::blue);
    
    Brain.Screen.drawRectangle(Vision.largestObject.originX * (500 / 640), Vision.largestObject.originY * (250 / 400), (Vision.largestObject.originX + Vision.largestObject.width) * (500 / 640), (Vision.largestObject.originY + Vision.largestObject.height) * (250 / 400));
    double initialX = Vision.largestObject.centerX;
    double expectedValue = 22;
    double errorSizeY =  expectedValue - Vision.largestObject.width;
    double errorDrift = initialX - Vision.largestObject.centerX;
    double speed = 80;
    double leftSpeed = speed; 
    double rightSpeed = speed;
    double kP = .5;
    
    while(errorSizeY != 0 || errorDrift != 0){
        Vision.takeSnapshot(BLUE_FLAG);
        errorSizeY =  expectedValue - Vision.largestObject.width;
        errorDrift = initialX - Vision.largestObject.centerX;
        leftSpeed -= errorDrift * kP; 
        rightSpeed += errorDrift * kP; 
        LD1.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
        LD2.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
        RD1.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
        RD2.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
        vex::task::sleep(20);
    }
    
    
    LD1.stop(); 
    RD1.stop(); 
    RD2.stop(); 
    LD2.stop();
    
    
    LD1.resetRotation();
    LD2.resetRotation(); 
    RD1.resetRotation(); 
    RD2.resetRotation();
    
}

void leds(){
    while(true){
        GREEN.set(false);
        RED.set(true);
        vex::task::sleep(200);
        YELLOW.set(true);
        RED.set(false);
        vex::task::sleep(200);
        GREEN.set(true);
        YELLOW.set(false);
        vex::task::sleep(200);
    }
}





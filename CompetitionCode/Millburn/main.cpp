#include "robot-config.h"


void userControl( void );
/*
*
*
*This is the function that we will be setting
*the drive control method for the competition to.
*
*
*/
void auton( void );
/*
*
*
*This is the function that we will be setting
*the autonomous method for the competition to.
*
*
*/
void drive();
/*
*
*
*This is the function that will contain the drive code.
*
*
*/
void shooter();
/*
*
*
*This is the function that will contain the shooter code.
*
*
*/
void arm();
/*
*
*
*This is the function that will contain the arm code.
*
*
*/
void intake();
/*
*
*
*This is the function that will contain the intake code.
*
*
*/
void pidDriveEncoder( double degrees, bool intakeOn, int dir );
/*
*
*
*This is the function that will be used to drive forward
*a certain amount of degrees forward using a pid loop 
*
*
*/
void pidTurn(double degrees);
/*
*
*turns using encoders and pid
*
*
*/
void blueFrontAuton();
/*
*
*function for the blue front auton
*
*
*/
void blueBackAuton();
/*
*
*function for the blue back auton
*
*
*/
void redBackAuton();
/*
*
*function for the red back auton
*
*
*/
void redFrontAuton();
/*
*
*function for the red front auton
*
*
*/
void shooterAutonomous();
/*
*
*shooter function for the auton
*
*
*/
void pidDriveVelocity(double degrees, bool intakeOn, int dir);
void pidDriveVelocityTwo(double degrees, bool intakeOn, int dir);
void regularDrive(double target);
void slowDrive(double degrees, int dir);
//void turn(double target, bool Dir, int speed);
//void driveTwo();

vex::thread shooterThread;
vex::thread intakeThread;

// pid loop variables
double error;
double kP;
double speed;
double integral;
double kI;
double kD;
double prevError;
double derivative;

double errorDrift; 
double kP_C; 
double proportionalDrift;
double leftSpeed; 
double rightSpeed; 
double kI_C; 
double integralDrift;
double kD_C; 
double prevErrorDrift;
double derivativeDrift;


double turnLeftNinety = -330;  
double turnRightNinety = 325; 
int autonSelect = 3;
bool platform = true; 
vex::competition Competition;
int main() {
    Competition.drivercontrol( userControl );
    Competition.autonomous( auton );
    
    bool pressed = false;
     
   while(!pressed){ 
        Brain.Screen.drawRectangle(0, 0, 250, 125, vex::color::red);
        Brain.Screen.drawRectangle(0, 125, 250, 250, vex::color::red);
        Brain.Screen.drawRectangle(250, 0, 500, 125, vex::color::blue);
        Brain.Screen.drawRectangle(250, 125, 500, 250, vex::color::blue);
        
        Brain.Screen.setPenColor(vex::color::white);
        Brain.Screen.printAt(80, 60, "Red Close");
        Brain.Screen.printAt(90, 185, "Red Far");
        Brain.Screen.printAt(315, 60, "Blue Close");
        Brain.Screen.printAt(320, 185, "Blue Far");
        
        if((Brain.Screen.pressing()) && (Brain.Screen.xPosition() >= 0 && Brain.Screen.xPosition() < 250) && (Brain.Screen.yPosition() >= 0 && Brain.Screen.yPosition() < 125)){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(50, 50, "Red Close Auton Selected!");
            autonSelect = 4;
            pressed = true;
        }
        else if((Brain.Screen.pressing()) && (Brain.Screen.xPosition() >= 250  && Brain.Screen.xPosition() < 500) && (Brain.Screen.yPosition() >= 0 && Brain.Screen.yPosition() < 125)){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(50, 50, "Blue Close Auton Selected!");
            autonSelect = 1;
            pressed = true;
        }
        else if((Brain.Screen.pressing()) && (Brain.Screen.xPosition() >= 250  && Brain.Screen.xPosition() < 500) && (Brain.Screen.yPosition() >= 125 && Brain.Screen.yPosition() < 250)){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(50, 50, "Blue Far Auton Selected!");
            autonSelect = 2;
            pressed = true;
        }
        else if((Brain.Screen.pressing()) && (Brain.Screen.xPosition() >= 0  && Brain.Screen.xPosition() < 250) && (Brain.Screen.yPosition() >= 125 && Brain.Screen.yPosition() < 250)){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(50, 50, "Red Far Auton Selected!");
            autonSelect = 3;
            pressed = true;
        }
        
        
        vex::task::sleep(200); 
    }
    
    
    bool secondPressed = false; 
    while(!secondPressed){
        Brain.Screen.drawRectangle(0, 0, 250, 250, vex::color::green);
        Brain.Screen.drawRectangle(250, 0, 500, 250, vex::color::orange);
        
        Brain.Screen.setPenColor(vex::color::white);
        Brain.Screen.printAt(100, 50, "Do you want to go on the platform?");
        Brain.Screen.printAt(80, 110, "YES");
        Brain.Screen.printAt(350, 110, "NO");
        
        if((Brain.Screen.pressing()) && (Brain.Screen.xPosition() >= 0 && Brain.Screen.xPosition() < 250)){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(50, 100, "PLATFORM");
            secondPressed = true;
            platform = true;
        }
        else if((Brain.Screen.pressing()) && (Brain.Screen.xPosition() >= 250 && Brain.Screen.xPosition() < 500)){
            Brain.Screen.clearScreen();
            Brain.Screen.printAt(50, 100, "NO PLATFORM");
            secondPressed = true;
            platform = false;
        }
        vex::task::sleep(200);
    }
    
    switch(autonSelect){
        case 1: 
            Brain.Screen.printAt(50, 50, "Blue Close Auton Selected!");
            break;  
        case 2: 
            Brain.Screen.printAt(50, 50, "Blue Far Auton Selected!");
            break; 
        case 3: 
            Brain.Screen.printAt(50, 50, "Red Far Auton Selected!");
            break; 
        case 4: 
            Brain.Screen.printAt(50, 50, "Red Close Auton Selected!");
            break;
    }
    
    
   
    while( true ){
        vex::task::sleep( 10 );
    }
   
    return 1;
}

void drive(){
    if(cont.ButtonUp.pressing()){
        LeftDriveMotorOne.spin( vex::directionType::fwd, -30, vex::velocityUnits::pct );
        LeftDriveMotorTwo.spin( vex::directionType::fwd, -30, vex::velocityUnits::pct );
        RightDriveMotorOne.spin( vex::directionType::fwd, -30, vex::velocityUnits::pct );
        RightDriveMotorTwo.spin( vex::directionType::fwd, -30, vex::velocityUnits::pct );
    }
    else if(cont.ButtonDown.pressing()){
        LeftDriveMotorOne.spin( vex::directionType::fwd, 30, vex::velocityUnits::pct );
        LeftDriveMotorTwo.spin( vex::directionType::fwd, 30, vex::velocityUnits::pct );
        RightDriveMotorOne.spin( vex::directionType::fwd, 30, vex::velocityUnits::pct );
        RightDriveMotorTwo.spin( vex::directionType::fwd, 30, vex::velocityUnits::pct );
    }
    else if(cont.ButtonRight.pressing()){
        LeftDriveMotorOne.spin( vex::directionType::fwd, 20, vex::velocityUnits::pct );
        LeftDriveMotorTwo.spin( vex::directionType::fwd, 20, vex::velocityUnits::pct );
        RightDriveMotorOne.spin( vex::directionType::fwd, -20, vex::velocityUnits::pct );
        RightDriveMotorTwo.spin( vex::directionType::fwd, -20, vex::velocityUnits::pct );
    }
    else if(cont.ButtonLeft.pressing()){
        LeftDriveMotorOne.spin( vex::directionType::fwd, -20, vex::velocityUnits::pct );
        LeftDriveMotorTwo.spin( vex::directionType::fwd, -20, vex::velocityUnits::pct );
        RightDriveMotorOne.spin( vex::directionType::fwd, 20, vex::velocityUnits::pct );
        RightDriveMotorTwo.spin( vex::directionType::fwd, 20, vex::velocityUnits::pct );
    }
    else if(cont.ButtonY.pressing()){
       pidDriveEncoder(-1250, false, 1);
       
    }
    else{
        LeftDriveMotorOne.spin( vex::directionType::fwd, -cont.Axis3.value() + cont.Axis1.value(), vex::velocityUnits::pct );
        LeftDriveMotorTwo.spin( vex::directionType::fwd, -cont.Axis3.value() + cont.Axis1.value(), vex::velocityUnits::pct );
        RightDriveMotorOne.spin( vex::directionType::fwd, -cont.Axis3.value() - cont.Axis1.value(), vex::velocityUnits::pct );
        RightDriveMotorTwo.spin( vex::directionType::fwd, -cont.Axis3.value() - cont.Axis1.value(), vex::velocityUnits::pct );
    }
}


void shooter(){
    while(true){
        if( cont.ButtonA.pressing() ){
            Shooter.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
            ShooterSecond.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
        }
        else if( cont.ButtonR2.pressing() ){
            int cur = Shooter.rotation(vex::rotationUnits::deg);
            int initial = cur;  
            while(cur < initial + 1803){
                cur = Shooter.rotation(vex::rotationUnits::deg);
                Shooter.spin(vex::directionType::fwd, 95, vex::velocityUnits::pct);
                ShooterSecond.spin(vex::directionType::fwd, 95, vex::velocityUnits::pct);
            }

        }
        else {
            Shooter.stop(vex::brakeType::coast);
            ShooterSecond.stop(vex::brakeType::coast);
        }
    }
}

void arm(){

}

void intake(){
    while(true){
        if( cont.ButtonR1.pressing() ){
            Intake.spin( vex::directionType::rev, 70, vex::velocityUnits::pct );
        }
        else if( cont.ButtonL1.pressing() ){
           Intake.spin( vex::directionType::fwd, 95, vex::velocityUnits::pct );
        }
        else{
            Intake.stop();
        }
    }
}

void userControl( void ){
    
    
    intakeThread = vex::thread(intake);
    shooterThread = vex::thread(shooter);
    while(true){
        drive();
    }
}

void pidDriveEncoder(double target, bool intakeOn, int dir){
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
    error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
    kP = .3;
    kI =.002;
    if(target > 0){
        kD = -0.3;//-.1
    }
    else{
        kD = 0.3;
    }
    kP_C = 0.70;
    kI_C = 0.01; 
    integralDrift = 0; 
    integral = 0;
    prevError = error;
    speed = 0;
    while(error != 0){
        error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
        if((abs(error) < 100) && (error != 0)){
            if(error > 0){
               integral += error; 
            }
            else{
               error = 0; 
            }
        }
        else{
            integral = 0;
        }
        derivative = error - prevError;
        prevError = error; 
        
        if(abs(error) < 800){
            speed = error * kP + integral * kI + derivative * kD;
        }
        else{
            if(target > 0){
                speed += 5;
            }
            else{
                speed -= 5;
            }
        }
        errorDrift = abs(LeftDriveMotorOne.rotation(vex::rotationUnits::deg)) - abs(RightDriveMotorOne.rotation(vex::rotationUnits::deg));
        Brain.Screen.printAt(50, 150, "%f", errorDrift);
        if(errorDrift != 0){// drifting to the right
            errorDrift = abs(LeftDriveMotorOne.rotation(vex::rotationUnits::deg)) - abs(RightDriveMotorOne.rotation(vex::rotationUnits::deg));
            proportionalDrift = errorDrift * kP_C;
            if(abs(errorDrift) < 20){
                integralDrift += errorDrift;
            }
            if(target > 0){
                leftSpeed = speed - (proportionalDrift + kI_C * integralDrift); 
                rightSpeed = speed + (proportionalDrift + kI_C * integralDrift);   
            }
            else if(target < 0){
                leftSpeed = speed + (proportionalDrift + kI_C * integralDrift); 
                rightSpeed = speed - (proportionalDrift + kI_C * integralDrift); 
            }


        }
        else{
            leftSpeed = speed; 
            rightSpeed = speed; 
        }
        Brain.Screen.printAt(50, 50, "Error: %f", error);
        Brain.Screen.printAt(50, 100, "Speed: %f", speed);
        

        
        LeftDriveMotorOne.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::rpm);
        LeftDriveMotorTwo.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::rpm);
        RightDriveMotorOne.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::rpm);
        RightDriveMotorTwo.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::rpm);
        if(intakeOn){
            Intake.spin(vex::directionType::rev, dir * 50, vex::velocityUnits::pct);
        }
        
        if(abs(error) < .5){
            error = 0; 
        }
        
        vex::task::sleep(20);
        
    }
    
    LeftDriveMotorOne.stop();
    LeftDriveMotorTwo.stop();
    RightDriveMotorOne.stop();
    RightDriveMotorTwo.stop();
    Intake.stop();
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();

}

void pidTurn(double target){
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
    
    error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
    kP = .3;
    kI = .002;
    if(target > 0){
        kD = -0.3;//-.1
    }
    else{
        kD = 0.3;
    } 
    
    integral = 0;
    prevError = error;
    speed = 0;
    while(error != 0){
        error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
        
        if(abs(error) < 325 && (error != 0)){
            if(target > 0){
                if(error> 0){
                    integral += error;
                }
                else{
                    error = 0; 
                }
            }
            else if(target < 0){
                if(error < 0){
                    integral += error; 
                }
                else{
                    error = 0; 
                }
            }
             
        }
        else{
            integral = 0;
        }
        derivative = error - prevError;
        prevError = error; 
        
        speed = error * kP + integral * kI + derivative * kD;
        
        Brain.Screen.printAt(50, 50, "Error: %f", error);
        Brain.Screen.printAt(50, 100, "Speed: %f", speed);
        

        
        LeftDriveMotorOne.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
        LeftDriveMotorTwo.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
        RightDriveMotorOne.spin(vex::directionType::fwd, -speed, vex::velocityUnits::rpm);
        RightDriveMotorTwo.spin(vex::directionType::fwd, -speed, vex::velocityUnits::rpm);
        
        vex::task::sleep(20);
        
    }
    
    LeftDriveMotorOne.stop();
    LeftDriveMotorTwo.stop();
    RightDriveMotorOne.stop();
    RightDriveMotorTwo.stop();
    
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();

}

void pidDriveVelocity(double target, bool intakeOn, int dir){
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
    error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
    kP = .3;
    kI =.002;
    if(target > 0){
        kD = -0.3;//-.1
    }
    else{
        kD = 0.3;
    }
    kP_C = 0.60;
    kI_C = .2; 
    integralDrift = 0; 
    integral = 0;
    prevError = error;
    speed = 0;
    while(error != 0){
        error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
        if((abs(error) < 100) && (error != 0)){
            if(error > 0){
               integral += error; 
            }
            else{
               error = 0; 
            }
        }
        else{
            integral = 0;
        }
        derivative = error - prevError;
        prevError = error; 
        
        if(abs(error) < 800){
            speed = error * kP + integral * kI + derivative * kD;
        }
        else{
            if(target > 0){
                speed += 3;
            }
            else{
                speed -= 3;
            }
        }
        errorDrift = abs(LeftDriveMotorOne.velocity(vex::velocityUnits::rpm)) - abs(RightDriveMotorOne.velocity(vex::velocityUnits::rpm));
        Brain.Screen.printAt(50, 150, "%f", errorDrift);
        if(errorDrift != 0){// drifting to the right
            errorDrift = abs(LeftDriveMotorOne.velocity(vex::velocityUnits::rpm)) - abs(RightDriveMotorOne.velocity(vex::velocityUnits::rpm));
            proportionalDrift = errorDrift * kP_C;
            if(abs(errorDrift) < 20 && error != 0){
                integralDrift += errorDrift;
            }
            else{
                integralDrift = 0; 
            }
            if(target > 0){
                leftSpeed = speed - (proportionalDrift + kI_C * integralDrift); 
                rightSpeed = speed + (proportionalDrift + kI_C * integralDrift);   
            }
            else if(target < 0){
                leftSpeed = speed + (proportionalDrift + kI_C * integralDrift); 
                rightSpeed = speed - (proportionalDrift + kI_C * integralDrift); 
            }


        }
        else{
            leftSpeed = speed; 
            rightSpeed = speed; 
        }
        Brain.Screen.printAt(50, 50, "Error: %f", error);
        Brain.Screen.printAt(50, 100, "Speed: %f", speed);
        

        
        LeftDriveMotorOne.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::rpm);
        LeftDriveMotorTwo.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::rpm);
        RightDriveMotorOne.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::rpm);
        RightDriveMotorTwo.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::rpm);
        if(intakeOn){
            Intake.spin(vex::directionType::rev, dir * 80, vex::velocityUnits::pct);
        }
        
        if(abs(error) < .5){
            error = 0; 
        }
        
        vex::task::sleep(20);
        
    }
    
    LeftDriveMotorOne.stop();
    LeftDriveMotorTwo.stop();
    RightDriveMotorOne.stop();
    RightDriveMotorTwo.stop();
    Intake.stop();
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();

}


void slowDrive(double degrees, int dir){
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
    
    int cur = LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
    int speed = 60;
    if(degrees < 0){
        speed = -speed;
    }
    while(abs(cur) < abs(degrees)){
        
        LeftDriveMotorOne.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        LeftDriveMotorTwo.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        RightDriveMotorOne.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        RightDriveMotorTwo.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
        cur = LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
        Intake.spin(vex::directionType::rev, dir * 100, vex::velocityUnits::pct);
    }
    
    LeftDriveMotorOne.stop();
    LeftDriveMotorTwo.stop();
    RightDriveMotorOne.stop();
    RightDriveMotorTwo.stop();
    Intake.stop();
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();   
}


void pidDriveVelocityTwo(double target, bool intakeOn, int dir){
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
    error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
    kP = .60;//.65
    kI =.0025;
    if(target > 0){
        kD = -0.5;//-.3
    }
    else{
        kD = 0.5;
    }
    kP_C = 4.0;
    kI_C = 1.65;
    if(target > 0){
        kD_C = -.5;
    }
    else{
        kD_C = .5;
    }
     
    derivativeDrift = 0; 
    integralDrift = 0; 
    integral = 0;
    prevError = error;
    prevErrorDrift = errorDrift;
    speed = 0;
    while(error != 0){
        error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
        if((abs(error) < 100) && (error != 0)){
            if(error > 0){
               integral += error; 
            }
            else{
               error = 0; 
            }
        }
        else{
            integral = 0;
        }
        derivative = error - prevError;
        prevError = error; 
        
        if(abs(error) <= 500){
            //go back to regular drift values
            kP_C = 0.70;//.67 
            kI_C = 0.040;  //0.01
            kD_C = 0; 
            speed = error * kP + integral * kI + derivative * kD;
        }
        else{
            if(target > 0){
                speed += 28;
            }
            else{
                speed -= 28;
            }
        }
        errorDrift = abs(LeftDriveMotorOne.velocity(vex::velocityUnits::rpm)) - abs(RightDriveMotorOne.velocity(vex::velocityUnits::rpm));
        Brain.Screen.printAt(50, 150, "%f", errorDrift);
        if(errorDrift != 0){// drifting to the right
            errorDrift = abs(LeftDriveMotorOne.velocity(vex::velocityUnits::rpm)) - abs(RightDriveMotorOne.velocity(vex::velocityUnits::rpm));
            proportionalDrift = errorDrift * kP_C;
            if(abs(errorDrift) < 35){
                integralDrift += errorDrift;
            }
            else{
                integralDrift = 0; 
            }
            
            derivativeDrift = errorDrift - prevErrorDrift; 
            prevErrorDrift = errorDrift; 
            if(target > 0){
                leftSpeed = speed - (proportionalDrift + kI_C * integralDrift + derivativeDrift * kD_C); 
                rightSpeed = speed + (proportionalDrift + kI_C * integralDrift + derivativeDrift * kD_C);   
            }
            else if(target < 0){
                leftSpeed = speed + (proportionalDrift + kI_C * integralDrift + derivativeDrift * kD_C); 
                rightSpeed = speed - (proportionalDrift + kI_C * integralDrift + derivativeDrift * kD_C); 
            }


        }
        else{
            leftSpeed = speed; 
            rightSpeed = speed; 
        }
        Brain.Screen.printAt(50, 50, "Error: %f", error);
        Brain.Screen.printAt(50, 100, "Speed: %f", speed);
        

        
        LeftDriveMotorOne.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::rpm);
        LeftDriveMotorTwo.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::rpm);
        RightDriveMotorOne.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::rpm);
        RightDriveMotorTwo.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::rpm);
        if(intakeOn){
            Intake.spin(vex::directionType::rev, dir * 80, vex::velocityUnits::pct);
        }
        
        if(abs(error) < .5){
            error = 0; 
        }
        
        vex::task::sleep(20);
        
    }
    
    LeftDriveMotorOne.stop();
    LeftDriveMotorTwo.stop();
    RightDriveMotorOne.stop();
    RightDriveMotorTwo.stop();
    Intake.stop();
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();

}


void shooterAutonomous(){
    int cur = Shooter.rotation(vex::rotationUnits::deg);
    int initial = cur;  
    while(cur < initial + 1803){
        cur = Shooter.rotation(vex::rotationUnits::deg);
        Shooter.spin(vex::directionType::fwd, 95, vex::velocityUnits::pct);
        ShooterSecond.spin(vex::directionType::fwd, 95, vex::velocityUnits::pct);
    }
    
    Shooter.stop();
    ShooterSecond.stop();
}

void regularDrive(double degrees, bool intakeOn, int dir){
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
    
    int cur = LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
    while(cur < degrees){
        LeftDriveMotorOne.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
        LeftDriveMotorTwo.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
        RightDriveMotorOne.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
        RightDriveMotorTwo.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
        cur = LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
        if(intakeOn){
            Intake.spin(vex::directionType::rev, dir * 50, vex::velocityUnits::pct);
        }
    }
    
    LeftDriveMotorOne.stop();
    LeftDriveMotorTwo.stop();
    RightDriveMotorOne.stop();
    RightDriveMotorTwo.stop();
    Intake.stop();
    
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
}



void auton( void ) {
    intakeThread.interrupt();
    shooterThread.interrupt();
    //select auton
    if(autonSelect == 1){
        blueFrontAuton();
    }
    else if(autonSelect == 2){
        blueBackAuton();
    }
    else if(autonSelect == 3){
        redBackAuton();
    }
    else if(autonSelect == 4){
        redFrontAuton();
    }
    
}

void blueFrontAuton(){

    slowDrive(-1250, 1);
    vex::task::sleep(200);
    slowDrive(1450, 1);
    vex::task::sleep(400);
    slowDrive(-50, 1);
    vex::task::sleep(200);
    pidTurn(-345);
    vex::task::sleep(400);
    slowDrive(-50, 1);
    vex::task::sleep(500);
    shooterAutonomous();
    if(platform){ 
        slowDrive(-450, 1);
        pidTurn(-340);
        regularDrive(1600, false, 1);
    }
    else{
        pidTurn(20);
        slowDrive(400, 1);
        pidTurn(-20);
        slowDrive(1200, 1);
        slowDrive(-500, 1);
    }
}



void blueBackAuton(){
    
    //drive forward to pick up ball
    //drive forward to pick up ball
    slowDrive(-1250, 1);
    
    //drive back a little bit
    slowDrive(260, 1);
    
    
    if(platform){
       //turn to face the platform
        pidTurn(-330);    
        //get on
        regularDrive(1075, false, 1);
    }
    
}

void redBackAuton(){
    
    //drive forward to pick up ball
    slowDrive(-1250, 1);
    
    //drive back a little bit
    slowDrive(260, 1);
    
    if(platform){
       //turn to face the platform
        pidTurn(320);

        regularDrive(1075, false, 1);
    }
    
}

void redFrontAuton(){
    slowDrive(-1250, 1);
    vex::task::sleep(200);
    slowDrive(1450, 1);
    vex::task::sleep(400);
    slowDrive(-50, 1);
    vex::task::sleep(200);
    pidTurn(348);
    vex::task::sleep(400);
    slowDrive(-40, 1);
    vex::task::sleep(500);
    shooterAutonomous();
    if(platform){ 
        slowDrive(-300, 1);
        pidTurn(330);
        regularDrive(1600, false, 1);
    }
    else{
        pidTurn(-20);
        slowDrive(400, 1);
        pidTurn(20);
        slowDrive(1200, 1);
        slowDrive(-500, 1);
    }
    /*
    Intake.spin(vex::directionType::fwd);
  //drive forward to pick up ball
    pidDriveVelocity(-1250, true, 1);
    
    //drive back to the starting tile
    pidDriveVelocity(1050, true, 1);

    //turn right to face flags
    pidTurn(310);
    
    vex::task::sleep(200);
    pidDriveVelocity(-300, false, 1);
    
    shooterAutonomous();
    

    if(platform){
        
       

        
        //drive back to reach the platform tiles
        pidDriveVelocity(-500, false, 1);
        
        //turn right
        pidTurn(330);
        
        //drive on the platty
        regularDrive(1600, false, 1);
    }
    
    else{
        
        pidTurn(-20);
        
        pidDriveVelocity(1400, true, 1);
       
    
        //drive back to spot
    
        pidDriveVelocity(-500, true, 1);
    
    }
    */


}



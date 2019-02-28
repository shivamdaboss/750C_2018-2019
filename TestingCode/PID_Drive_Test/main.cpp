#include "robot-config.h"
#include <cmath>
          
double error_Left;
double error_Right;
double difference;

//proportional part
double kP_Left;
double kP_Right;
double speed_Left;
double speed_Right;

//integral part
double dT;
double integral_Left;
double integral_Right;
double kI_Left;
double kI_Right;
double integralLimit;
double higherIntegralLimit;

//derivative part
double prevError_Left;
double prevError_Right;
double derivative_Left;
double derivative_Right;
double kD_Right;
double kD_Left;

//turning pid loop variables
double error;
double kP;
double speed;
double integral;
double kI; 
double prevError; 
double derivative;   
double kD;  
double gyroVal;
void driveForward(double target);

int main() {
    Gyro.startCalibration(500);
    vex::task::sleep(500);
    driveForward(1000);
}


void driveForward(double target){
    LeftDriveOne.resetRotation();
    LeftDriveTwo.resetRotation();
    RightDriveOne.resetRotation();
    RightDriveTwo.resetRotation();
    
    
    Brain.Screen.printAt(50, 50, "LIT");
    error_Left = target - (LeftDriveOne.rotation(vex::rotationUnits::deg) + LeftDriveTwo.rotation(vex::rotationUnits::deg))/2;
    error_Right = target - (RightDriveOne.rotation(vex::rotationUnits::deg) + RightDriveTwo.rotation(vex::rotationUnits::deg))/2;
    
    difference = error_Left - error_Right;
    
    kP_Left = 1;
    kP_Right = 1;
    
    kI_Left = 0;
    kI_Right = 0;
    
    kD_Left = 0;
    kD_Right = 0;
    
    
    integralLimit = 300;
    higherIntegralLimit = 0;
    dT = 0.015;
    
    prevError_Left = 0;
    prevError_Right = 0;
    
    while( (error_Left != 0) && (error_Right != 0)){
        Brain.Screen.printAt(50, 50, "%f", error_Left);
        Brain.Screen.printAt(50, 100, "%f", error_Right);
        //calculating errors for both sides of the drives
        error_Left = target - (LeftDriveOne.rotation(vex::rotationUnits::deg) + LeftDriveTwo.rotation(vex::rotationUnits::deg))/2;
        error_Right = target - (RightDriveOne.rotation(vex::rotationUnits::deg) + RightDriveTwo.rotation(vex::rotationUnits::deg))/2;
        difference = error_Left - error_Right;
        
        if(error_Left < 0){
            
        }
        
        //calculating the integral factors for both sides of the drive
       /* integral_Left += error_Left * dT;
        integral_Right += error_Right * dT;

        //the limitations on the left integral
        if(error_Left == 0){
            integral_Left = 0;
        }
        if(!(error_Left <= integralLimit) && (std::abs(error_Left) <= higherIntegralLimit)){
            integral_Left = 0;
        }
           
        //the limitations on the right integral
        if(error_Right == 0){
            integral_Right = 0;
        }
        if(!(error_Right <= integralLimit) && (std::abs(error_Right) >= higherIntegralLimit)){
            integral_Right = 0;
        }
           
        //derivative part left
        
        derivative_Left = error_Left - prevError_Left;
        prevError_Left = error_Left;
        
        //derivative part right

        derivative_Right = error_Right - prevError_Right;
        prevError_Right = error_Right;
*/
        //speed calculation
        speed_Left = (error_Left * kP_Left) + (integral_Left * kI_Left) + (derivative_Left * kD_Left) - difference;
        speed_Right = (error_Right * kP_Right) + (integral_Right * kI_Right) + (derivative_Right * kD_Right) + difference;
        
        if(speed_Left >= 200){
            speed_Left = 200;
        }
        else if(speed_Left <= -200){ 
            speed_Left = -200;
        }
        if(speed_Right >= 200){
            speed_Right = 200;
        }
        else if(speed_Right <= -200){
            speed_Right = -200;
        }
        
        Brain.Screen.printAt(50, 150, "Left: %f", (LeftDriveOne.rotation(vex::rotationUnits::deg) + LeftDriveTwo.rotation(vex::rotationUnits::deg))/2);
        Brain.Screen.printAt(50, 200, "Right: %f",  (RightDriveOne.rotation(vex::rotationUnits::deg) + RightDriveTwo.rotation(vex::rotationUnits::deg))/2);
        
        LeftDriveOne.spin(vex::directionType::fwd, speed_Left, vex::velocityUnits::rpm);
        LeftDriveTwo.spin(vex::directionType::fwd, speed_Left, vex::velocityUnits::rpm);
        RightDriveOne.spin(vex::directionType::fwd, speed_Right, vex::velocityUnits::rpm);
        RightDriveTwo.spin(vex::directionType::fwd, speed_Right, vex::velocityUnits::rpm);
        
        vex::task::sleep(dT * 1000);
    }
          
    LeftDriveOne.stop(vex::brakeType::brake);
    LeftDriveTwo.stop(vex::brakeType::brake);
    RightDriveOne.stop(vex::brakeType::brake);
    RightDriveTwo.stop(vex::brakeType::brake);
    
    LeftDriveOne.resetRotation();
    LeftDriveTwo.resetRotation();
    RightDriveOne.resetRotation();
    RightDriveTwo.resetRotation();
    
}

/*void turnLeft(int degrees){
    gyroVal =  Gyro.value(vex::rotationUnits::deg);
    error = degrees - gyroVal;
    kP = 0;
    kI = 0;  
    kD = 0; 
    integralLimit = 300;
    higherIntegralLimit = 0; 
    dT = 0.015;
    prevError = 0;
    while(degrees != gyroVal ){
        gyroVal = Gyro.value(vex::rotationUnits::deg);
        error =degrees - gyroVal;
        
        integral +=  error * dT;
        
        if(!(error <= integralLimit) && !(std::abs(error) <= higherIntegralLimit)){
            integral = 0;
        }
        derivative = error - prevError;
        prevError =error;
        
        
        speed = (error * kP) + (integral * kI) + (derivative * kD);
        
        LeftDriveOne.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
        LeftDriveTwo.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
        RightDriveOne.spin(vex::directionType::rev, speed, vex::velocityUnits::rpm);
        RightDriveTwo.spin(vex::directionType::rev, speed, vex::velocityUnits::rpm);
        
        vex::task::sleep(dT * 1000);
    }
    
    LeftDriveOne.stop(vex::brakeType::brake);
    LeftDriveTwo.stop(vex::brakeType::brake);
    RightDriveOne.stop(vex::brakeType::brake);
    RightDriveTwo.stop(vex::brakeType::brake);
    
    LeftDriveOne.resetRotation();
    LeftDriveTwo.resetRotation();
    RightDriveOne.resetRotation();
    RightDriveTwo.resetRotation();
    
    
}*/


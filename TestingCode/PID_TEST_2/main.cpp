#include "robot-config.h"
          
double error;
double kP;
double speed;
double integral;
double kI;
double kD;
double prevError;
double derivative;
void pidDriveGyro(double target);
void pidTurn(double degrees);
void pidDriveEncoder(double degrees, bool intakeOn, int dir);
void pidDriveVelocity(double degrees, bool intakeOn, int dir); 
void pidDriveVelocityTwo(double degrees, bool intakeOn, int dir);

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


int main() {
    Gyro.startCalibration(500);
    while(Gyro.isCalibrating()){
        vex::task::sleep(20);
    }
    pidDriveVelocity(1500, false, 1);
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



void pidDriveGyro(double target){
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
    error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
    double initialGyroVal = Gyro.value(vex::rotationUnits::deg);
    double currentGyroVal = Gyro.value(vex::rotationUnits::deg);
    kP = .3;
    kI =.002;
    if(target > 0){
        kD = -0.3;//-.1
    }
    else{
        kD = 0.3;
    }
    kP_C = 1.5;
    kI_C = 0;
    integral = 0;
    prevError = error;
    speed = 0;
    while(error != 0){
        error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
        if(abs(error) < 100 && (error != 0)){
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
                speed += 20;
            }
            else{
                speed -= 20;
            }
        }
        currentGyroVal = Gyro.value(vex::rotationUnits::deg);
        Brain.Screen.printAt(50, 150, "%f", currentGyroVal);
        if(currentGyroVal > initialGyroVal){// drifting to the right
            errorDrift = abs(abs(currentGyroVal) - abs(initialGyroVal));
            proportionalDrift = errorDrift * kP_C;
            if(target > 0){
                leftSpeed = speed + (proportionalDrift ); 
                rightSpeed = speed - (proportionalDrift );   
            }
            else if(target < 0){
                leftSpeed = speed - (proportionalDrift ); 
                rightSpeed = speed + (proportionalDrift ); 
            }


        }
        
        else if(currentGyroVal < initialGyroVal ){// drifting to the left
            errorDrift = abs(abs(currentGyroVal) - abs(initialGyroVal));
            proportionalDrift = errorDrift * kP_C;
            if(target > 0){
                leftSpeed = speed + (proportionalDrift ); 
                rightSpeed = speed - (proportionalDrift );   
            }
            else if(target < 0){
                leftSpeed = speed - (proportionalDrift ); 
                rightSpeed = speed + (proportionalDrift ); 
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

void pidTurnGyro(double target){
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
        
    error = target - Gyro.value(vex::rotationUnits::deg);
    kP = 10;
    kI = 0.002;
    /*if(target > 0){
        kD = -0.3;//-.1
    }
    else{
        kD = 0.3;
    } */
    
    integral = 0;
    prevError = error;
    speed = 0;
    double gyroValue = Gyro.value(vex::rotationUnits::deg);
    while(error != 0){
        error = target - gyroValue;
        Brain.Screen.printAt(50, 150, "Gyro: %f", gyroValue);
        gyroValue = Gyro.value(vex::rotationUnits::deg);
        
        if(fabs(error) < 10 && (error != 0)){
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
                speed += 10;
            }
            else{
                speed -= 10;
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
            //Intake.spin(vex::directionType::rev, dir * 80, vex::velocityUnits::pct);
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
    //Intake.stop();
    
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
    kP_C = 0.9;//.67
    kI_C = 0;//.01 
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
                speed += 10;
            }
            else{
                speed -= 10;
            }
        }
        errorDrift = abs(LeftDriveMotorOne.velocity(vex::velocityUnits::rpm)) - abs(RightDriveMotorOne.velocity(vex::velocityUnits::rpm));
        Brain.Screen.printAt(50, 150, "%f", errorDrift);
        if(errorDrift != 0){// drifting to the right
            errorDrift = abs(LeftDriveMotorOne.velocity(vex::velocityUnits::rpm)) - abs(RightDriveMotorOne.velocity(vex::velocityUnits::rpm));
            proportionalDrift = errorDrift * kP_C;
            if(abs(errorDrift) < 30){
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
            //Intake.spin(vex::directionType::rev, dir * 80, vex::velocityUnits::pct);
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
    //Intake.stop();
    
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
        kD = 0.3;
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
            //Intake.spin(vex::directionType::rev, dir * 80, vex::velocityUnits::pct);
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
    //Intake.stop();
    
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();

}

#include "robot-config.h"
//variables needed for the main distance PID loop          
double error;
double kP;
double speed;
double integral;
double kI;
double kD;
double prevError;
double derivative;
double TARGET_GLOBAL;


//pid functions
void pidTurn(double target);
void pidDrive(double target);
void pidMatchVelocity(vex::motor& master, vex::motor& slave, double target);
void pidMatchVelocityLDT();
void pidMatchVelocityRDO();
void pidMatchVelocityRDT();
void pidInitiate();
int main() {
    Gyro.startCalibration(500);
    while(Gyro.isCalibrating()){
        vex::task::sleep(20);
    }
    pidDrive(1000);
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

void pidDrive(double target){
    TARGET_GLOBAL = target; 
    LeftDriveMotorOne.resetRotation();
    LeftDriveMotorTwo.resetRotation();
    RightDriveMotorOne.resetRotation();
    RightDriveMotorTwo.resetRotation();
    
    vex::thread LDT;
    vex::thread RDO; 
    vex::thread RDT;

    bool stopped = false;
    bool started = true;
    error = target - LeftDriveMotorOne.rotation(vex::rotationUnits::deg);
    kP = .3;
    kI =.002;
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
        if((fabs(error) < 100) && (error != 0)){
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
        
        if(fabs(error) < 400){
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
        Brain.Screen.printAt(50, 50, "Error: %f", error);
        Brain.Screen.printAt(50, 100, "Speed: %f", speed);

        LeftDriveMotorOne.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
        if(fabs(error) < 100){
            if(!stopped){
                LDT.interrupt();
                RDO.interrupt();
                RDT.interrupt();
                stopped = true;
            }
            
            LeftDriveMotorTwo.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
            RightDriveMotorOne.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
            RightDriveMotorTwo.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
            
        }
        
        if(target - error <= 50){
            LeftDriveMotorTwo.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
            RightDriveMotorOne.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
            RightDriveMotorTwo.spin(vex::directionType::fwd, speed, vex::velocityUnits::rpm);
        }
        else{
            if(started){
                vex::thread LDT = vex::thread(pidMatchVelocityLDT);
                vex::thread RDO = vex::thread(pidMatchVelocityRDO);
                vex::thread RDT = vex::thread(pidMatchVelocityRDT);        
                started = false;
            }    
        
        }
                
        vex::task::sleep(200);
        
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
void pidMatchVelocityLDT(){
    pidMatchVelocity(LeftDriveMotorOne, LeftDriveMotorTwo, TARGET_GLOBAL);
}

void pidMatchVelocityRDO(){
    pidMatchVelocity(LeftDriveMotorOne, RightDriveMotorOne, TARGET_GLOBAL);
}

void pidMatchVelocityRDT(){
    pidMatchVelocity(LeftDriveMotorOne, RightDriveMotorTwo, TARGET_GLOBAL);
    
}
void pidMatchVelocity(vex::motor& master, vex::motor& slave, double target){
    slave.spin(vex::directionType::fwd, master.velocity(vex::velocityUnits::rpm), vex::velocityUnits::rpm);
    double errorMain = target - master.rotation(vex::rotationUnits::deg);
    double errorVelocity = master.velocity(vex::velocityUnits::rpm) - slave.velocity(vex::velocityUnits::rpm);
    double P = .5; 
    double I = .10; 
    double D = -5;
    double speed = 0;
    double prevErrorVelocity = errorVelocity;
    double Derivative = errorVelocity - prevErrorVelocity;
    double Integral = 0;
    while(true){
        errorMain = target - master.rotation(vex::rotationUnits::deg);
        errorVelocity = master.velocity(vex::velocityUnits::rpm) - slave.velocity(vex::velocityUnits::rpm);
        Derivative = prevErrorVelocity - errorVelocity;
        prevErrorVelocity = errorVelocity;
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(50, 50, "MASTER SPEED: %f", master.velocity(vex::velocityUnits::rpm));
        Brain.Screen.printAt(50, 100, "SLAVE SPEED: %f", slave.velocity(vex::velocityUnits::rpm));
        Brain.Screen.printAt(50, 150, "DIFFERENCE: %f", master.velocity(vex::velocityUnits::rpm) - slave.velocity(vex::velocityUnits::rpm));
        
        if(fabs(errorVelocity) < 20 && errorVelocity != 0){
           Integral += errorVelocity;
        }
        else{
            Integral = 0;
        }
        
        speed = (P * errorVelocity) + (I * Integral) + (D * Derivative);
        slave.spin(vex::directionType::fwd, slave.velocity(vex::velocityUnits::rpm) + speed, vex::velocityUnits::rpm);
        vex::task::sleep(10);
    }
}




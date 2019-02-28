#include "robot-config.h"
          

int main() {
    Gyro.startCalibration(500);
    Brain.Screen.printAt(50, 50, "Finished Calibrating!");
    vex::task::sleep(500);
    Brain.Screen.clearScreen();
    double gyroValue = Gyro.value(vex::rotationUnits::deg);
    while(true){
        Brain.Screen.printAt(50, 50, "%f %", gyroValue);
        gyroValue = Gyro.value(vex::rotationUnits::deg);
        vex::task::sleep(200);
    }
}

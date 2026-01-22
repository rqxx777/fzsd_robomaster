#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/VacuumGripper.hpp>
#include <cmath>
#include <iostream>

using namespace webots;
using namespace std;

int main() {
    Robot* robot = new Robot();
    int timeStep = static_cast<int>(robot->getBasicTimeStep());
    
    Motor* motors[6];
    motors[0] = robot->getMotor("motor1");
    motors[1] = robot->getMotor("motor2");
    motors[2] = robot->getMotor("motor3");
    motors[3] = robot->getMotor("motor4");
    motors[4] = robot->getMotor("motor5");
    motors[5] = robot->getMotor("motor6");
    
    VacuumGripper* gripper = robot->getVacuumGripper("gripper");
    
    auto waitSeconds = [&](double seconds) {
        int steps = static_cast<int>(seconds * 1000.0 / timeStep);
        for (int i = 0; i < steps; ++i) {
            robot->step(timeStep);
        }
    };
    
    auto setJointAngles = [&](const double angles[6]) {
        for (int i = 0; i < 6; ++i) {
            if (motors[i]) {
                motors[i]->setPosition(angles[i]);
            }
        }
    };
    
    auto calculateInverseKinematics = [](double targetX, double targetY, double targetZ, double resultAngles[6]) -> bool {
        const double baseHeight = 0.3;
        const double upperArmLength = 0.3;
        const double forearmLength = 0.2;
        const double wristLength = 0.3;
        const double endEffectorLength = 0.77;
        
        resultAngles[0] = atan2(targetY, targetX);
        
        double q1 = resultAngles[0];
        double targetPitch = 0.0;
        double targetYaw = 0.0;
        
        double c1 = cos(q1);
        double s1 = sin(q1);
        double cp = cos(targetPitch);
        double sp = sin(targetPitch);
        double cy = cos(targetYaw);
        double sy = sin(targetYaw);
        
        double wx = targetX - endEffectorLength * (cy*cp*c1 - sy*s1);
        double wy = targetY - endEffectorLength * (cy*cp*s1 + sy*c1);
        double wz = targetZ - baseHeight - endEffectorLength * cy*sp;
        
        double r_wrist = sqrt(wx*wx + wy*wy);
        double z_wrist = wz;
        
        double D = (r_wrist*r_wrist + z_wrist*z_wrist - 
                   upperArmLength*upperArmLength - 
                   forearmLength*forearmLength) / 
                  (2.0 * upperArmLength * forearmLength);
        
        if (D > 1.0 || D < -1.0) {
            return false;
        }
        
        resultAngles[2] = acos(D);
        
        double phi = atan2(z_wrist, r_wrist);
        double psi = atan2(forearmLength * sin(resultAngles[2]),
                          upperArmLength + forearmLength * cos(resultAngles[2]));
        resultAngles[1] = phi - psi;
        
        double q23 = resultAngles[1] + resultAngles[2];
        double c23 = cos(q23);
        double s23 = sin(q23);
        
        double R13_temp = -s23;
        double R23_temp = c23;
        double R33_temp = 0.0;
        
        resultAngles[3] = atan2(R23_temp, R13_temp);
        
        double R31_temp = cy*sp*c1 + sy*s1;
        double R32_temp = cy*sp*s1 - sy*c1;
        double R33_final = cy*cp;
        
        double sin_q5 = sqrt(R31_temp*R31_temp + R32_temp*R32_temp);
        resultAngles[4] = atan2(sin_q5, R33_final);
        
        double cos_q4 = R13_temp/cos(resultAngles[3]);
        double sin_q4 = R23_temp/cos(resultAngles[3]);
        resultAngles[5] = atan2(-R32_temp/cos(resultAngles[4]), R31_temp/cos(resultAngles[4]));
        
        double q1_test = resultAngles[0], q2_test = resultAngles[1], q3_test = resultAngles[2];
        double q4_test = resultAngles[3], q5_test = resultAngles[4], q6_test = resultAngles[5];
        
        double c1t = cos(q1_test), s1t = sin(q1_test);
        double c2t = cos(q2_test), s2t = sin(q2_test);
        double c3t = cos(q3_test), s3t = sin(q3_test);
        double c4t = cos(q4_test), s4t = sin(q4_test);
        double c5t = cos(q5_test), s5t = sin(q5_test);
        double c6t = cos(q6_test), s6t = sin(q6_test);
        
        double c23t = cos(q2_test + q3_test);
        double s23t = sin(q2_test + q3_test);
        
        double calculatedX = c1t*(upperArmLength*c2t + forearmLength*c23t + wristLength*s23t + 
                                endEffectorLength*(c5t*c23t - s5t*s23t));
        double calculatedY = s1t*(upperArmLength*c2t + forearmLength*c23t + wristLength*s23t + 
                                endEffectorLength*(c5t*c23t - s5t*s23t));
        double calculatedZ = baseHeight + upperArmLength*s2t + forearmLength*s23t - 
                           wristLength*c23t + endEffectorLength*(c5t*s23t + s5t*c23t);
        
        double positionError = sqrt(pow(calculatedX - targetX, 2) + 
                                   pow(calculatedY - targetY, 2) + 
                                   pow(calculatedZ - targetZ, 2));
        
        if (positionError > 0.01) {
            return false;
        }
        
        return true;
    };
    
    double firstPosition[6] = {0.0, 1.2200, 0.6300, 0.0, 1.2900, 0.0};
    double secondPosition[6] = {M_PI, 1.2200, 0.6300, 0.0, 1.2900, 0.0};
    
    setJointAngles(firstPosition);
    waitSeconds(3.0);
    
    if (gripper) {
        gripper->turnOn();
    }
    waitSeconds(1.0);
    
    setJointAngles(secondPosition);
    waitSeconds(2.0);
    
    if (gripper) {
        gripper->turnOff();
    }
    waitSeconds(0.5);
    
    delete robot;
    return 0;
}

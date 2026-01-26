#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <cmath>
#include <string>

using namespace webots;

bool calculateIK(double targetX, double targetY, double targetZ, double angles[6]) {
    const double d1 = 0.2;
    const double a2 = 0.4;
    const double d4 = 0.6;
    const double d6 = 0.77;
    
    angles[0] = atan2(targetY, targetX);
    
    double wristX = targetX - d6;
    double wristY = targetY;
    double wristZ = targetZ;
    
    double dx = wristX;
    double dy = wristY;
    double dz = wristZ - d1;
    
    double r = sqrt(dx*dx + dy*dy);
    double D = sqrt(r*r + dz*dz);
    
    if (D > (a2 + d4) || D < fabs(a2 - d4)) {
        return false;
    }
    
    double cos_q3 = (a2*a2 + d4*d4 - D*D) / (2.0 * a2 * d4);
    if (cos_q3 > 1.0) cos_q3 = 1.0;
    if (cos_q3 < -1.0) cos_q3 = -1.0;
    angles[2] = acos(cos_q3);
    
    double alpha = atan2(dz, r);
    double beta = asin(d4 * sin(angles[2]) / D);
    angles[1] = alpha - beta;
    
    angles[3] = 0.0;
    angles[4] = 0.0;
    angles[5] = 0.0;
    
    return true;
}

int main() {
    Robot* robot = new Robot();
    int timeStep = static_cast<int>(robot->getBasicTimeStep());
    
    auto wait = [&](double seconds) {
        int steps = static_cast<int>(seconds * 1000.0 / timeStep);
        for (int i = 0; i < steps; ++i) {
            robot->step(timeStep);
        }
    };
    
    Motor* motors[6];
    for (int i = 0; i < 6; i++) {
        motors[i] = robot->getMotor("motor" + std::to_string(i+1));
    }
    
    auto setJoint = [&](int joint, double angle) {
        if (motors[joint]) {
            motors[joint]->setPosition(angle);
        }
    };
    
    double targetX = 0.4;
    double targetY = 0.0;
    double targetZ = 0.6;
    
    double angles[6];
    if (calculateIK(targetX, targetY, targetZ, angles)) {
        for (int i = 0; i < 6; i++) {
            setJoint(i, angles[i]);
        }
        wait(2.0);
        
        setJoint(4, M_PI/6);
        wait(0.3);
        
        setJoint(4, 0.0);
        wait(1.0);
    }
    
    while (robot->step(timeStep) != -1) {
    }
    
    delete robot;
    return 0;
}

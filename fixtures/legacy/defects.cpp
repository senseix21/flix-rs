#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <cstring>
#include <csignal>
#include <termios.h>
#include <cstdio>
#include "Arduino.h"
extern float t, dt;
#include "vector.h"
#include "quaternion.h"
Vector rates;
Quaternion attitude;
float motors[4] = {};
float controlRoll = 0, controlPitch = 0, controlYaw = 0, controlThrottle = 0;
float controlMode = NAN, controlTime = NAN;
const int MOT_RL = 0, MOT_RR = 1, MOT_FR = 2, MOT_FL = 3;
void failsafe();
void rcLossFailsafe();
void autoFailsafe();
void tiltFailsafe();
void descend();
void interpretControls();
void controlAttitude();
void controlRates();
void controlTorque();
void desaturate();
void computeLoopRate();
#include "control.ino"
#include "safety.ino"
#include "time.ino"
int main() {
    t = 20; dt = 0.001f;
    mode = AUTO; armed = false;
    torqueTarget.invalidate();
    for (auto& motor : motors) motor = 0.4f;
    controlTorque();
    assert(motors[0] == 0.4f);
    std::printf("Disarmed + invalid torque: motor remains %.2f\n", motors[0]);
    armed = true; controlTime = NAN; thrustTarget = 0.6f;
    rcLossFailsafe();
    assert(thrustTarget == 0.6f);
    std::printf("AUTO + unset controlTime: thrust remains %.2f\n", thrustTarget);
    controlTime = 0; thrustTarget = 0.6f;
    rcLossFailsafe(); controlTorque();
    assert(thrustTarget < 0.6f && motors[0] == 0.4f);
    std::printf("Direct output + expired controlTime: target %.4f, motor %.2f\n", thrustTarget, motors[0]);
    Rate rate(10); t = 4294; assert(rate); t = 0.001f;
    assert(!rate);
    std::printf("Rate after clock rollback: blocked (last=%.1f, now=%.3f)\n", rate.last, t);
}

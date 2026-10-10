#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <cstring>
#include <csignal>
#include <termios.h>
#include <cstdio>
// The pinned legacy Arduino.h calls atol/atof without including <cstdlib>;
// current libstdc++/libc++ no longer provide it transitively.
#include <cstdlib>
// Legacy vector.h/quaternion.h call unqualified isfinite; libstdc++ declares it
// only in std::, libc++ also globally.
#include <cmath>
using std::isfinite;
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

#include <sstream>
#include <iostream>
#include <vector>
int main() {
 std::string line;
 while(std::getline(std::cin,line)) {
  std::istringstream row(line);std::string token;std::vector<float> v;
  std::getline(row,token,',');const std::string kind=token;
  while(std::getline(row,token,',')) v.push_back(std::stof(token));
  if(kind=="mix") {
   armed=true;thrustTarget=v.at(0);torqueTarget=Vector(v.at(1),v.at(2),v.at(3));controlTorque();
   for(int i=0;i<4;i++) std::printf("%s%.9f",i?",":"",std::clamp(motors[i],0.0f,1.0f));
  } else if(kind=="quaternion") {
   auto q=Quaternion::fromEuler(Vector(v.at(0),v.at(1),v.at(2)));
   std::printf("%.9f,%.9f,%.9f,%.9f",q.w,q.x,q.y,q.z);
  } else if(kind=="control") {
   t=1;dt=.001;controlTime=t;armed=true;mode=(int)v.at(10);controlMode=NAN;
   controlRoll=v.at(0);controlPitch=v.at(1);controlYaw=v.at(2);controlThrottle=v.at(3);
   attitude=Quaternion::fromEuler(Vector(v.at(4),v.at(5),v.at(6)));rates=Vector(v.at(7),v.at(8),v.at(9));
   attitudeTarget=Quaternion::fromEuler(Vector(0,0,attitude.getYaw()));ratesTarget=Vector();ratesExtra=Vector();torqueTarget=Vector();
   rollRatePID.reset();pitchRatePID.reset();yawRatePID.reset();rollPID.reset();pitchPID.reset();yawPID.reset();
   control();for(int i=0;i<4;i++) std::printf("%s%.9f",i?",":"",std::clamp(motors[i],0.0f,1.0f));
  } else return 2;
  std::puts("");
 }
}

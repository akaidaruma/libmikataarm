#include <iostream>
#include <stdlib.h>
#include "mikata.h"
#include "kinematics.h"
#include "Thread.h"

using namespace ssr::mikata;

int main(const int argc, const char* argv[]) {
  if (argc < 3) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./test filename baudrate" << std::endl;
    return -1;
  }


  try {
    MikataArm mikataArm(argv[1], atoi(argv[2]));
    mikataArm.servoOn(false);
    std::vector<double> joints;
    std::vector<JointInfo> js = mikataArm.jointInfos();
    joints.clear();
    for(int i = 0; i < numJoints; i++){
      std::cout << "joint[" << i << "]" << js[i].angle << std::endl;
      joints.push_back(js[i].angle);
    }
    Matrix44 m = forward_kinematics(joints);

    std::cout << "Transformation Matrix:" << std::endl << str(m) << std::endl;

    double x = atof(argv[3]);
    double y = atof(argv[4]);
    double z = atof(argv[5]);

    std::string buf;
    std::cout << "X, Y, Z:" << x << "," << y << "," << z <<std::endl;
    std::cout << "OK?" << std::ends;
    std::cin >> buf;
    std::cout << "buf = " << buf << std::endl;
    if (buf.c_str()[0] != 'y' && buf.c_str()[0] != 'Y'){
      return -1;
    }
    mikataArm.servoOn(true);

    Matrix44 mm;
    mm.v[0][0] = -1.0; mm.v[0][1] = 0; mm.v[0][2] =  0.0; mm.v[0][3] = x;
    mm.v[1][0] =  0.0; mm.v[1][1] = 1; mm.v[1][2] =  0.0; mm.v[1][3] = y;
    mm.v[2][0] =  0.0; mm.v[2][1] = 0; mm.v[2][2] = -1.0; mm.v[2][3] = z;
    mm.v[3][0] =  0.0; mm.v[3][1] = 0; mm.v[3][2] =  0.0; mm.v[3][3] = 1;

    std::vector<double> solved = inverse_kinematics(mm);
    std::vector<JointCommand> jcs;
    for(int i = 0; i < numJoints; i++){
      std::cout << "solved joint[" << i << "]" << solved[i] << std::endl;
      JointCommand jc;
      jc.angle = solved[i];
      jcs.push_back(jc);
    }
    mikataArm.move(jcs);
    mikataArm.servoOn(false);

  } catch (std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    return -1;
  }


  //std::cout << "Mikata Arm Move Example" << std::endl;

  return 0;
};

#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include "Strategy/VisionAlignment.h"
#include "ControlMap.h"

using namespace photonlib;

VisionAlignment::VisionAlignment(std::string name, Drivetrain &drivetrain, bool track) : wml::Strategy(name), _drivetrain(drivetrain), _drivetrainAngleStrategy("VisionAngle", drivetrain, _lastYaw), _track(track){
  Requires(&drivetrain);
  SetCanBeInterrupted(true);
}

void VisionAlignment::OnStart() {
  _drivetrainAngleStrategy.OnStart();
}

//drivetrain snap strat 
void VisionAlignment::OnUpdate(double dt) {
  double leftPower = 0, rightPower = 0;

  // photonCamera.photonLib::SetLED
  std::cout << photonCamera.GetLEDMode() << std::endl;
  photonCamera.SetLEDMode(LEDMode::kOn);

  double xCords = _visionTable->GetEntry("targetPixelsX").GetDouble(0); 
  double yCords = _visionTable->GetEntry("targetPixelsY").GetDouble(0);
  double yawCords = _visionTable->GetEntry("targetYaw").GetDouble(0);
  double gyro = _drivetrain.GetConfig().gyro->GetAngle();
  double isFinished = _visionTable->GetEntry("Is finished").SetBoolean(_drivetrainAngleStrategy.IsFinished());

  // 3.21m 

  if (std::abs(yawCords - _lastYaw) > 0.005)
    _drivetrainAngleStrategy.SetGoal(gyro + yawCords);

  _drivetrainAngleStrategy.OnUpdate(dt);

  if (!_track) {
    if (_drivetrainAngleStrategy.IsFinished())
      SetDone();
  }

  // nt::NetworkTableInstance::GetDefault().GetTable("testVisionTable")->GetEntry("lastYaw").SetDouble(_lastYaw);

  std::cout << "yawCord: " << yawCords << std::endl;
  std::cout << "gyro: " << gyro << std::endl;

  _lastYaw = yawCords;
}


// -------- Shooting and Distance Vision Stuff -------- 


VisionSnapStrat::VisionSnapStrat(std::string name) : wml::Strategy(name) {
  SetCanBeInterrupted(true);
  SetCanBeReused(true);
  // Requires(&vision);
  // SetPassive(true);
  std::cout << "vision snap strat" << std::endl;
}

void VisionSnapStrat::OnUpdate(double dt) {
  // std::cout << "Fuck" << std::endl;
  // auto inst = nt::NetworkTableInstance::GetDefault();
  // auto snapTable = inst.GetTable("Snap vision stuff");
  // // snapTable->GetEntry("isOnTarget").SetBoolean(isInnerCircle);

  // double pitch = _visionTable->GetEntry("targetPitch").GetDouble(0);

  // double newSpeed = (fixSpeed2-fixSpeed1)/(fixPitch2-fixPitch1)*(pitch-fixPitch1)+fixSpeed1;
  // snapTable->GetEntry("newSpeed").SetDouble(newSpeed);


  // if (pitch <= -18 && pitch >= -22) {
  //   isInnerCircle = true;
  //   std::cout << "inner target" << std::endl;
  // } else {
  //   isInnerCircle = false;
  // }
} //-20 inner circle
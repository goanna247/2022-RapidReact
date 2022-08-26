#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include "Strategy/VisionAlignment.h"
#include "ControlMap.h"
#include "cmath"

using namespace photonlib;

VisionAlignment::VisionAlignment(std::string name, Drivetrain &drivetrain, bool track, Controllers &contGroup) : wml::Strategy(name), _drivetrain(drivetrain), _drivetrainAngleStrategy("VisionAngle", drivetrain, _lastYaw), _track(track), _contGroup(contGroup){
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
  // std::cout << photonCamera.GetLEDMode() << std::endl;
  // photonCamera.SetLEDMode(LEDMode::kOn);

  double xCords = _visionTable->GetEntry("targetPixelsX").GetDouble(0); 
  double yCords = _visionTable->GetEntry("targetPixelsY").GetDouble(0);
  double yawCords = _visionTable->GetEntry("targetYaw").GetDouble(0);
  double gyro = _drivetrain.GetConfig().gyro->GetAngle();
  double isFinished = _visionTable->GetEntry("Is finished").SetBoolean(_drivetrainAngleStrategy.IsFinished());

  // 3.21m 

  if (std::abs(yawCords - _lastYaw) > 0.005)
    _drivetrainAngleStrategy.SetGoal((gyro + yawCords) - 5);

  _drivetrainAngleStrategy.OnUpdate(dt);

  if (!_track) {
    if (_drivetrainAngleStrategy.IsFinished())
      SetDone();
  }

  // nt::NetworkTableInstance::GetDefault().GetTable("testVisionTable")->GetEntry("lastYaw").SetDouble(_lastYaw);

  std::cout << "yawCord: " << yawCords << std::endl;
  std::cout << "gyro: " << gyro << std::endl;

  if (_contGroup.Get(ControlMap::visionCancel, wml::controllers::XboxController::ONRISE)) {
    Interrupt();
  }

  _lastYaw = yawCords;
}


VisionAutoAlignment::VisionAutoAlignment(std::string name, Drivetrain &drivetrain) : wml::Strategy(name), _drivetrain(drivetrain), _drivetrainAngleStrategy("VisionAngle", drivetrain, _lastYaw) {
  Requires(&drivetrain);
  SetCanBeInterrupted(true);
}

void VisionAutoAlignment::OnStart() {
  // _drivetrainAngleStrategy.OnStart();
}

void VisionAutoAlignment::OnUpdate(double dt) {
  // double leftPower = 0, rightPower = 0;
  // VisionAlignmentTimer.Start();

  // double xCords = _visionTable->GetEntry("targetPixelsX").GetDouble(0); 
  // double yCords = _visionTable->GetEntry("targetPixelsY").GetDouble(0);
  // double yawCords = _visionTable->GetEntry("targetYaw").GetDouble(0);
  // double gyro = _drivetrain.GetConfig().gyro->GetAngle();
  // double isFinished = _visionTable->GetEntry("Is finished").SetBoolean(_drivetrainAngleStrategy.IsFinished());

  // // 3.21m 

  // if (std::abs(yawCords - _lastYaw) > 0.005)
  //   _drivetrainAngleStrategy.SetGoal((gyro + yawCords) - 10);

  // _drivetrainAngleStrategy.OnUpdate(dt);

  // if (VisionAlignmentTimer.Get() >= 5_s) {
  //   VisionAlignmentTimer.Stop();
  //   VisionAlignmentTimer.Reset();
  //   SetDone();
  // } 

  // // if (!_track) {
  //   if (_drivetrainAngleStrategy.IsFinished())
  //     SetDone();
  // // }

  // // nt::NetworkTableInstance::GetDefault().GetTable("testVisionTable")->GetEntry("lastYaw").SetDouble(_lastYaw);

  // std::cout << "yawCord: " << yawCords << std::endl;
  // std::cout << "gyro: " << gyro << std::endl;

  // _lastYaw = yawCords;
}

VisionDistance::VisionDistance(std::string name, Shooter &shooter) : wml::Strategy(name), _shooter(shooter) {
  SetCanBeInterrupted(true);
}

void VisionDistance::OnStart() {

}

void VisionDistance::OnUpdate(double dt) {
  double launchAngle = 70;
  double pitch = _visionTable->GetEntry("pitch").GetDouble(0); 
  double tapeHeight = 2.605;
  double ringHeight = 2.5;

  double distanceToRing = 0.679+tapeHeight/tan(pitch*3.1416/180);  // Centre of ring in meters
  double exitVelocity = pow(((-4.9*pow(distanceToRing, 2)) / (pow(cos(launchAngle*3.1416/180), 2))) / (ringHeight-tan(launchAngle*3.1416/180)*distanceToRing), 0.5);
  double offSetFactor = 1.1; // of set function maybe linear?? Might be wrong
  double exitAngularVelocity = ((exitVelocity/((3.1416*4*25.4)/1000)) * (2*3.1416)) * offSetFactor;  // radians per second (times exitAngularVelocity by of set value)

  _shooter.calculatePID(exitAngularVelocity, dt);

}
#pragma once

#include <iostream>
#include "Vision.h"
#include "controllers/Controller.h"
#include "RobotMap.h"
#include "RobotControl.h"
#include "control/MotorFilters.h"
#include "DriveToDistanceStrategy.h"
#include "Shooter.h"

// TODO: Use PhotonCamera from RobotMap
class VisionAlignment : public wml::Strategy {
 public:
  VisionAlignment(std::string name, Drivetrain &drivetrain, bool track);

  void OnStart() override;
  void OnUpdate(double dt) override;

 private:
  Drivetrain &_drivetrain;
  std::shared_ptr<nt::NetworkTable> _visionTable = nt::NetworkTableInstance::GetDefault().GetTable("photonvision/visionCam");
  DrivetrainAngleStrategy _drivetrainAngleStrategy;
  double _lastYaw = 0;
  double _accSpeed = 0.2;
  bool _track = false;
};  // moves robot to align with tape


class VisionDistance : public wml::Strategy {
 public:
  VisionDistance(std::string name, Shooter &shooter);

  void OnStart() override;
  void OnUpdate(double dt) override;

 private:
  Shooter &_shooter;
  std::shared_ptr<nt::NetworkTable> _visionTable = nt::NetworkTableInstance::GetDefault().GetTable("photonvision/visionCam");
};
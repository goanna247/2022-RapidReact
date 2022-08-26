#pragma once

#include "strategy/Strategy.h"
#include "Drivetrain.h"
#include "Intake.h"
#include "Shooter.h"
// #include "Robot.h"

using Controllers = wml::controllers::SmartControllerGroup;

class Auto {
 public:
  std::shared_ptr<wml::Strategy> FiveBallTerminal(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
  std::shared_ptr<wml::Strategy> ThreeBallTerminal(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
  std::shared_ptr<wml::Strategy> ThreeBallHanger(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
  std::shared_ptr<wml::Strategy> OneTwoBallAuto(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter, Controllers &contGroup);
  std::shared_ptr<wml::Strategy> TurningTest(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter, Controllers &contGroup);
  std::shared_ptr<wml::Strategy> Vision(wml::Drivetrain &drivetrain , Controllers &contGroup);
  std::shared_ptr<wml::Strategy> Distance();
  std::shared_ptr<wml::Strategy> DriveTest(wml::Drivetrain &drivetrain, Intake &intake, Shooter &shooter);
};
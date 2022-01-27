#include "Robot.h"
#include <iostream>

using namespace frc;
using namespace wml;

double currentTimeStamp;
double lastTimeStamp;
double dt;

// General Robot Logic
void Robot::RobotInit() {
  //Init the controllers
  ControlMap::InitSmartControllerGroup(robotMap.contGroup);

  shooter = new Shooter(robotMap.shooterSystem, robotMap.contGroup);
  robotMap.shooterSystem.leftFlyWheelMotor.SetInverted(true);
  robotMap.shooterSystem.rightFlyWheelMotor.SetInverted(true);
  robotMap.shooterSystem.centerFlyWheelMotor.SetInverted(true);

}

void Robot::RobotPeriodic() {
  currentTimeStamp = frc::Timer::GetFPGATimestamp().value();
  dt = currentTimeStamp - lastTimeStamp;

  StrategyController::Update(dt);
  NTProvider::Update();

  lastTimeStamp = currentTimeStamp;
}

// Disabled Logic
void Robot::DisabledInit() {
  InterruptAll(true);
}
void Robot::DisabledPeriodic() {}

// Auto Robot Logic
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

// Manual Robot Logic
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  shooter->teleopOnUpdate(dt);
}

// During Test Logic
void Robot::TestInit() {}
void Robot::TestPeriodic() {} 
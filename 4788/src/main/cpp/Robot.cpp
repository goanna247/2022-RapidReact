#include "Robot.h"

using namespace frc;
using namespace wml;

double currentTimeStamp;
double lastTimeStamp;
double dt;

// General Robot Logic
void Robot::RobotInit() {

	//Init the controllers
	ControlMap::InitSmartControllerGroup(robotMap.contGroup);

	shooter = new Shooter(robotMap.shooterSystem.leftFlyWheelMotor, robotMap.shooterSystem.rightFlyWheelMotor, robotMap.contGroup);
	robotMap.shooterSystem.leftFlyWheelMotor.SetInverted(true);
	robotMap.shooterSystem.rightFlyWheelMotor.SetInverted(true);

	drivetrain = new wml::Drivetrain(robotMap.driveSystem.drivetrainConfig, robotMap.driveSystem.gainsVelocity);
	robotMap.driveSystem.drivetrain.GetConfig().leftDrive.encoder->ZeroEncoder();
	robotMap.driveSystem.drivetrain.GetConfig().rightDrive.encoder->ZeroEncoder();

	drivetrain->SetDefault(std::make_shared<DrivetrainManual>("Drivetrain Manual", *drivetrain, robotMap.contGroup));
	drivetrain->StartLoop(100);

	drivetrain->GetConfig().rightDrive.transmission->SetInverted(false);
	drivetrain->GetConfig().leftDrive.transmission->SetInverted(true);

	intake = new Intake(robotMap.intakeSystem.leftIntakeActuation, robotMap.intakeSystem.rightIntakeActuation, robotMap.intakeSystem.intakeMotor, robotMap.contGroup);
	robotMap.intakeSystem.intakeMotor.SetInverted(false);

	climber = new Climber(robotMap.climberSystem.leftClimber, robotMap.climberSystem.rightClimber, robotMap.contGroup);

	StrategyController::Register(drivetrain);
	NTProvider::Register(drivetrain);
}
void Robot::RobotPeriodic() {
	currentTimeStamp = (double)frc::Timer::GetFPGATimestamp();
	dt = currentTimeStamp - lastTimeStamp;

	StrategyController::Update(dt);

	// robotMap.controlSystem.compressor.SetTarget(wml::actuators::BinaryActuatorState::kForward);
	// robotMap.controlSystem.compressor.Update(dt);

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
void Robot::TeleopInit() {
	Schedule(drivetrain->GetDefaultStrategy(), true);
}
void Robot::TeleopPeriodic() {
	shooter->teleopOnUpdate(dt);
	intake->teleopOnUpdate(dt);
	climber->teleopOnUpdate(dt);
}

// During Test Logic
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
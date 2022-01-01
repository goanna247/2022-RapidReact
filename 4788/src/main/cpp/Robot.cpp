#include "Robot.h"

using namespace frc;
using namespace wml;

using hand = frc::XboxController::JoystickHand;

double currentTimeStamp;
double lastTimeStamp;
double dt;

double rightPower = 0;
double leftPower = 0;

double motorSpeed = 0;
double speedIntake = 0;

double constexpr deadzone = 0.05;

void Robot::RobotInit() {
	Driver = new frc::XboxController(0);
	CoDriver = new frc::XboxController(1);

	//drivebase 
	_frontLeftMotor = new wml::TalonSrx(3);
	_backLeftMotor = new wml::TalonSrx(4);

	_frontRightMotor = new wml::TalonSrx(6);
	_backRightMotor = new wml::TalonSrx(7);

	//intake 
	_intakeMotor = new wml::TalonSrx(1);

	//hammer time 
	_hammerMotor = new wml::TalonSrx(2);

	_intakeMotor->SetInverted(true);
	_hammerMotor->SetInverted(false);

	_frontLeftMotor->SetInverted(true);
	_backLeftMotor->SetInverted(true);

	_frontRightMotor->SetInverted(false);
	_backRightMotor->SetInverted(false);
}
void Robot::RobotPeriodic() {}


void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

// Manual Robot Logic
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
	currentTimeStamp = Timer::GetFPGATimestamp();
	dt = currentTimeStamp - lastTimeStamp;

	//its hammer time 
	//go forwards with the left trigger 
	motorSpeed = CoDriver->GetTriggerAxis(hand::kLeftHand);
	if (motorSpeed >= deadzone) {
		_hammerMotor->Set(0.8);
	} else {
		_hammerMotor->Set(0);
	}

	//go backwards with the y button 
	if (CoDriver->GetYButton()) {
		_hammerMotor->Set(-0.5);
	}

	//drive base, tank drive
	if (fabs(Driver->GetY(hand::kRightHand)) >= deadzone) {
		leftPower = Driver->GetY(hand::kRightHand);
	} else {
		leftPower = 0;
	}

	if (fabs(Driver->GetY(hand::kLeftHand)) >= deadzone ) {
		rightPower = Driver->GetY(hand::kLeftHand);
	} else {
		rightPower = 0;
	}

	//intake, right trigger
	speedIntake = CoDriver->GetTriggerAxis(hand::kRightHand);
	if (speedIntake >= deadzone) {
		_intakeMotor->Set(1);
	} else {
		_intakeMotor->Set(0);
	}

	//drivebase 
	_frontLeftMotor->Set(leftPower);
	_backLeftMotor->Set(leftPower);
	
	_frontRightMotor->Set(rightPower);
	_backRightMotor->Set(rightPower);

	lastTimeStamp = currentTimeStamp;
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
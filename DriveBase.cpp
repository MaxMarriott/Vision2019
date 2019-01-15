//==============================================================================
// DriveBase.cpp
//==============================================================================

#include "WPILib.h"
#include <Joystick.h>
#include <SmartDashboard/SmartDashboard.h>

#include "RobotConfiguration.h"
//#include <ctre/Phoenix.h>
//#include <ctrlib/PigeonIMU.h>
#include "DriveBase.h"

//==============================================================================
// Construction

DriveBase::DriveBase() : frc::Subsystem("DriveBase") {
//    TSingleton<DriveBase>(this),
//    m_left_master_speed_controller(NULL),
//    m_left_slave_speed_controller(NULL),
//    m_right_master_speed_controller(NULL),
//    m_right_slave_speed_controller(NULL),
//    m_robot_drive(NULL),
//    m_joystick(NULL),
//    m_pigen_imu(NULL),
//    m_dummy_left_speed_controller(NULL),
//    m_dummy_right_speed_controller(NULL),
//    m_log_string(),
//    m_log_counter(0) {
    m_joystick = NULL;
    m_pigen_imu = NULL;
    m_left_master_speed_controller = NULL;
    m_left_slave_speed_controller = NULL;
    m_right_master_speed_controller = NULL;
    m_right_slave_speed_controller = NULL;

    SetupSimple();
}

DriveBase::~DriveBase() {
    DriveBase::ShutdownSimple();
}

//==============================================================================
// frc::Subsystem Function Overrides

//void DriveBase::InitDefaultCommand() {
//    SetDefaultCommand(new DriveWithJoystick());
//}


//==============================================================================
// Setup and Shutdown

void DriveBase::SetupSimple() {
	// Create a joystick object for the joystick connected to the laptop
    m_joystick = new frc::Joystick(0);

    // Create an object to monitor the Pigeon gyro/compass/accelerometer
    m_pigen_imu = new PigeonIMU(RobotConfiguration::kPigeonImuId);

    // Create controllers for each of the 4 talons connected to the robot drive motors
    m_left_master_speed_controller = new TalonSRX(RobotConfiguration::kLeftMasterTalonId);
    m_left_slave_speed_controller = new TalonSRX(RobotConfiguration::kLeftSlaveTalonId);
    m_right_master_speed_controller = new TalonSRX(RobotConfiguration::kRightMasterTalonId);
    m_right_slave_speed_controller = new TalonSRX(RobotConfiguration::kRightSlaveTalonId);

    // Link slaves to their masters. Controlling the master will then control the slave automatically
    m_left_slave_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kLeftMasterTalonId);
    m_right_slave_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kRightMasterTalonId);

    // Set the peak and nominal voltages for the controllers
    m_left_master_speed_controller->ConfigNominalOutputForward(+0.0f, -0.0f);
    m_left_master_speed_controller->ConfigPeakOutputForward(+1.0f, -1.0f);
    m_right_master_speed_controller->ConfigNominalOutputForward(+0.0f, -0.0f);
    m_right_master_speed_controller->ConfigPeakOutputForward(+1.0f, -1.0f);

}

void DriveBase::ShutdownSimple() {
    // Delete the joystick and pigeon objects that we created, and clear the pointers
    delete m_joystick;
    m_joystick = NULL;
    delete m_pigen_imu;
    m_pigen_imu = NULL;

    // Delete the speed controller objects that we created and clear the pointers to them
    delete m_left_master_speed_controller;
    m_left_master_speed_controller = NULL;
    delete m_left_slave_speed_controller;
    m_left_slave_speed_controller = NULL;
    delete m_right_master_speed_controller;
    m_right_master_speed_controller = NULL;
    delete m_right_slave_speed_controller;
    m_right_slave_speed_controller = NULL;
}

//==============================================================================
// Operation

// Drive the robot using movement and rotation parameters.
//   -1.0 < moveValue < 1.0        positive values move forward
//   -1.0 < rotateValue < 1.0      positive values rotate clockwise
// If moveValue is not zero, the robot moves and rotates at the same time.
void DriveBase::ArcadeDrive(double moveValue, double rotateValue){

	// local variables to hold the computed PWM values for the motors
	double leftMotorOutput;
	double rightMotorOutput;

	if (moveValue > 0.0) {
	    if (rotateValue > 0.0) {
	      leftMotorOutput = moveValue - rotateValue;
	      rightMotorOutput = std::max(moveValue, rotateValue);
	    } else {
	      leftMotorOutput = std::max(moveValue, -rotateValue);
	      rightMotorOutput = moveValue + rotateValue;
	    }
	  } else {
	    if (rotateValue > 0.0) {
	      leftMotorOutput = -std::max(-moveValue, rotateValue);
	      rightMotorOutput = moveValue + rotateValue;
	    } else {
	      leftMotorOutput = moveValue - rotateValue;
	      rightMotorOutput = -std::max(-moveValue, -rotateValue);
	    }
	  }
	// Drive the motors directly. The right motor is reversed, so we need to change the sign.
	m_left_master_speed_controller->Set(ControlMode::PercentOutput, leftMotorOutput);
	m_right_master_speed_controller->Set(ControlMode::PercentOutput, -rightMotorOutput);

}

void DriveBase::Stop() {
    DriveBase::ArcadeDrive(0.0, 0.0);
}

double DriveBase::GetPigeonHeading() {
    // If there is no Pigeon IMU return an error
    if (m_pigen_imu == NULL) return kHeadingError;

    // If the Pigeon IMU is not ready return an error
    if (m_pigen_imu->GetState() != PigeonIMU::Ready) return kHeadingError;

    // The Pigeon has multiple methods for determining a heading.
    // For the moment we are using the 'fused' heading.
    return m_pigen_imu->GetFusedHeading();
}

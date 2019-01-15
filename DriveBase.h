//==============================================================================
// DriveBase.h
//==============================================================================

#ifndef SRC_DRIVEBASE_H_
#define SRC_DRIVEBASE_H_

#include <string>
#include <Commands/Subsystem.h>

// Include in header because it uses many nested namespaces which are difficult for forward declaration.
#include <ctre/Phoenix.h>

//class TalonSRX;
//class PigeonImu;
namespace frc
{
class Joystick;
class Talon;
class RobotDrive;
class Solenoid;
}

// DriveBase controls everything to do with the robot drive base, including:
// - Motor speed controllers
// - RobotDrive object
// - Joystick object
//
// Implements the 'ArcadeDrive' drive system.
class DriveBase : public frc::Subsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
    DriveBase();

    // Destructor
    virtual ~DriveBase();

    //==========================================================================
    // frc::Subsystem Function Overrides
    //virtual void InitDefaultCommand() override;
    //==========================================================================

    //==========================================================================
    // Setup and Shutdown

    // Setup the drive base for robot operation, without encoders
    void SetupSimple();

    // Shutdown the drivebase
    void ShutdownSimple();

    //==========================================================================
    // Operation

    // Drive the robot using movement and rotation parameters.
    //   -1.0 < moveValue < 1.0        positive values move forward
    //   -1.0 < rotateValue < 1.0      positive values rotate clockwise
    // If moveValue is not zero, the robot moves and rotates at the same time.
    void ArcadeDrive(double move, double rotate);

    // Stops the drive base
    void Stop();

private:
    double GetPigeonHeading();

    //==========================================================================
    // Member Variables

    TalonSRX* m_left_master_speed_controller;       // Left master Talon SRX speed controller
    TalonSRX* m_left_slave_speed_controller;        // Left slave Talon SRX speed controller
    TalonSRX* m_right_master_speed_controller;      // Right master Talon SRX speed controller
    TalonSRX* m_right_slave_speed_controller;       // Right slave Talon SRX speed controller
    frc::Joystick* m_joystick;                      // Joystick
    PigeonIMU* m_pigen_imu;                         // Pigeon IMU (inertial measurement unit)
    static constexpr double kHeadingError = 999.0;  // Value indicating an error reading the heading from the Pigeon IMU
};

#endif /* SRC_DRIVEBASE_H_ */

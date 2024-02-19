#pragma once

#include <rev/cansparkmax.h>
#include <Constants.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <numbers>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>


class Arm
{
public:
    typedef enum arm_positon_e
    {
        HOLD_START_POSITION,
        GROUND_PICKUP,
        SOURCE,
        AMP,
        SPEAKER,
        TRAP
    } arm_position_t;

    typedef enum wrist_positon_e
    {
        WRIST_GROUND_PICKUP,
        WRIST_SOURCE,
        WRIST_SHOOT,
        WRIST_AMP,
        WRIST_TRAP
    } wrist_position_t;

    Arm();

    void initArm();
    void disableArm();

    void SetArmPosition (arm_position_t DesiredPosition);
    void updateArm (/*Arm? I have those!*/);

    void UpdateSmartDashboardData();

    void armManualControl( double speed );
    void wristManualControl( double speed );
private:
    arm_position_t   m_ArmPosition   { arm_position_t::HOLD_START_POSITION };
    wrist_position_t m_WristPosition { wrist_position_t::WRIST_GROUND_PICKUP };

    double m_startArmAngle;
    double m_startWristAngle;

    // Arm
    rev::CANSparkMax      m_ArmMotorLeft  { ConstantCrap::kArmMotorLeftcanID,  rev::CANSparkLowLevel::MotorType::kBrushless };
    rev::CANSparkMax      m_ArmMotorRight { ConstantCrap::kArmMotorRightcanID, rev::CANSparkLowLevel::MotorType::kBrushless };
    
    rev::SparkPIDController m_pidControllerLeft  = m_ArmMotorLeft.GetPIDController();
    rev::SparkPIDController m_pidControllerRight = m_ArmMotorRight.GetPIDController();

    // default PID coefficients
    double kP = 0.2, kI = 0.0, kD = 0, kIz = 0, kFF = 0.000, kMaxOutput = 0.4, kMinOutput = -0.4;

    // default smart motion coefficients
    double kMaxVel = 0.8, kMinVel = 0, kMaxAcc = 2.2, kAllErr = 0;


    // default PID coefficients
    double kP2 = 0.2, kI2 = 0.0, kD2 = 0, kIz2 = 0, kFF2 = 0.000, kMaxOutput2 = 0.4, kMinOutput2 = -0.4;

    // default smart motion coefficients
    double kMaxVel2 = 1.8, kMinVel2 = 0, kMaxAcc2 = 1.2, kAllErr2 = 0;


    rev::SparkRelativeEncoder m_ArmMotorLeftEncoder  { m_ArmMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42) };
    rev::SparkRelativeEncoder m_ArmMotorRightEncoder { m_ArmMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42) };

    
    frc::DutyCycleEncoder m_ArmEncoder    { ConstantCrap::kArmEncoderDIO };

    // Wrist
    rev::CANSparkMax      m_WristMotor   { ConstantCrap::kWristMotorID, rev::CANSparkLowLevel::MotorType::kBrushless };
    frc::DutyCycleEncoder m_WristEncoder { ConstantCrap::kWristEncoderDIO };
    rev::SparkRelativeEncoder m_WristMotorEncoder  { m_WristMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42) };
    rev::SparkPIDController m_WristPidController  = m_WristMotor.GetPIDController();

    // ARM PID
    static constexpr auto kArmAngleVelocity =
      std::numbers::pi * 1_rad_per_s; // TODO : Determine this
    static constexpr auto kArmAngleAcceleration =
      std::numbers::pi * 2_rad_per_s / 1_s; // TODO : Determine this
     

    //rev::SparkPIDController m_armPidController2 = m_ArmMotorLeft.GetPIDController();

#if 0 
    // TODO : Determine these
    frc::ProfiledPIDController<units::radians> m_ArmPIDController{
      1.0, 
      0.0,
      0.0,
      {kArmAngleVelocity, kArmAngleAcceleration}};

    // WRIST PID
    static constexpr auto kWristAngleVelocity =
      std::numbers::pi * 1_rad_per_s; // TODO : Determine this
    static constexpr auto kWristAngleAcceleration =
      std::numbers::pi * 2_rad_per_s / 1_s; // TODO : Determine this
     
    // TODO : Determine these
    frc::ProfiledPIDController<units::radians> m_WristPIDController{
      1.0,
      0.0,
      0.0,
      {kWristAngleVelocity, kWristAngleAcceleration}};
#endif
};
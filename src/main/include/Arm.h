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
#include "units/angular_acceleration.h"


#define WRIST_USE_MOTOR_ENCODER ( 0 )

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
    bool ArmHold ();
    void SetArmPosition (arm_position_t DesiredPosition);
    void updateArm (/*Arm? I have those!*/);

    void UpdateSmartDashboardData();
    bool ArmReadyForGroundIntake();
    void armManualControl( double speed );
    void wristManualControl( double speed );
private:
    double getWristEncoderValue();

    arm_position_t   m_ArmPosition     { arm_position_t::HOLD_START_POSITION };
    arm_position_t   m_PrevArmPosition { arm_position_t::HOLD_START_POSITION };

    double m_startArmAngle;
    double m_startWristAngle;

    // Arm
    rev::CANSparkMax      m_ArmMotorLeft  { ConstantCrap::kArmMotorLeftcanID,  rev::CANSparkLowLevel::MotorType::kBrushless };
    rev::CANSparkMax      m_ArmMotorRight { ConstantCrap::kArmMotorRightcanID, rev::CANSparkLowLevel::MotorType::kBrushless };
    
    rev::SparkPIDController m_pidControllerLeft  = m_ArmMotorLeft.GetPIDController();
    rev::SparkPIDController m_pidControllerRight = m_ArmMotorRight.GetPIDController();




    rev::SparkRelativeEncoder m_ArmMotorLeftEncoder  { m_ArmMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor) };
    rev::SparkRelativeEncoder m_ArmMotorRightEncoder { m_ArmMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor) };

    
    frc::DutyCycleEncoder m_ArmEncoder    { ConstantCrap::kArmEncoderDIO };

    // Wrist
    rev::CANSparkMax      m_WristMotor   { ConstantCrap::kWristMotorID, rev::CANSparkLowLevel::MotorType::kBrushless };
    frc::DutyCycleEncoder m_WristEncoder { ConstantCrap::kWristEncoderDIO };
    rev::SparkRelativeEncoder m_WristMotorEncoder  { m_WristMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor) };
    rev::SparkPIDController m_WristPidController  = m_WristMotor.GetPIDController();

    double m_WristEncoderOffset = 0.0;

    // TODO : Determine these
    double m_ArmGroundValue     = 0.52;          
    double m_ArmSourceValue     = 0.32;          
    double m_ArmSpeakerValue    = 0.51;           
    double m_ArmAmpValue        = 0.25;       
    double m_ArmTrapValue       = 0.3;
    double m_ArmMaxOutputValue  = 0.8;             
    double m_ArmP               = 0.2;
    double m_ArmMaxVel          = 1.8;     
    double m_ArmMaxAccel        = 4.2;
    double m_ArmAngle           = 0.0;       


    // default PID coefficients
    double kP = m_ArmP, kI = 0.0, kD = 0, kIz = 0, kFF = 0.000, kMaxOutput = m_ArmMaxOutputValue, kMinOutput = -m_ArmMaxOutputValue;

    // default smart motion coefficients
    double kMaxVel = m_ArmMaxVel, kMinVel = 0, kMaxAcc = m_ArmMaxAccel, kAllErr = 0;




#if WRIST_USE_MOTOR_ENCODER
    // default PID coefficients
    double kP2 = 0.2, kI2 = 0.0, kD2 = 0, kIz2 = 0, kFF2 = 0.000, kMaxOutput2 = 0.4, kMinOutput2 = -0.4;

    // default smart motion coefficients
    double kMaxVel2 = 1.8, kMinVel2 = 0, kMaxAcc2 = 1.2, kAllErr2 = 0;
#else
    // TODO : Determine these
    double m_WristGroundValue    = 1.160;            
    double m_WristSourceValue    = 1.1;            
    double m_WristSpeakerValue   = 0.600;             
    double m_WristAmpValue       = 1.350;  
    double m_WristTrapValue      = 0.600;    
    double m_WristMaxOutputValue = 0.600;               
    double m_WristP              = 3.000;  
    double m_WristMaxVel         = double{std::numbers::pi * 1_rad_per_s};
    double m_WristMaxAccel       = double{std::numbers::pi * 2_rad_per_s / 1_s};
    double m_WristControlOutput = 0;
    double m_WristAngle = 0;
    frc::ProfiledPIDController<units::radians> m_WristPIDController{
      m_WristP,
      0.0,
      0.0,
      {units::radians_per_second_t{m_WristMaxVel}, units::radians_per_second_squared_t{m_WristMaxAccel}}};
#endif


};
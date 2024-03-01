#pragma once

#include <Phoenix5.h>
#include <Constants.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "units/angular_acceleration.h"


#define CLIMBER_MANUAL_CONTROL       ( 0 )
#define CLIMBER_ENCODER_SYNC_ENABLED ( 0 )
#define CLIMBER_ENCODER_SYNC_TRACK_L ( 0 )
#define CLIMBER_GYRO_ENABLED         ( 0 )


class Climber
{
 public:
    typedef enum ClimberState_e
    {
        ClimberDown,
        ClimberUp,
        ClimberStop
    } ClimberState_t;

    void initClimber();
    void updateClimber (/*Me when the me when... *Literally combusts* */);
    void ChangeClimberState( ClimberState_t ClimberState );
    void manualControl( double speedL, double speedR );
    void UpdateSmartDashboardData();
    void UpdateRoll( double roll );

 private:
    ClimberState_t m_ClimberState {ClimberState_t::ClimberStop};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_leftClimberMotor { ConstantCrap::kLeftClimberMotor };
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_rightClimberMotor{ ConstantCrap::kRightClimberMotor };

    frc::Encoder m_motorEncoderL { ConstantCrap::kLeftClimberEncoderDIO1,  ConstantCrap::kLeftClimberEncoderDIO2  };
    frc::Encoder m_motorEncoderR { ConstantCrap::kRightClimberEncoderDIO1, ConstantCrap::kRightClimberEncoderDIO2 };

    frc::DigitalInput m_rightLimitSwitch{ConstantCrap::kRightClimberStopDIO}; 
    frc::DigitalInput m_leftLimitSwitch {ConstantCrap::kLeftClimberStopDIO};

    double m_prevRoll = 0;
    double m_roll     = 0;


    static constexpr double kMaxClimberHeight = 2.9e5;

  #if CLIMBER_ENCODER_SYNC_ENABLED
    double m_ClimberPosition = 0.0;

    // Number of encoder counts to change climber position each update.
    double kCLimberSpeed = 1000.0;
    double kAngleToEncoderCounts = 0.0;

    double m_ClimberLMaxOutputValue = 1.000;               
    double m_ClimberLP              = 0.00015;  

    frc::PIDController m_ClimberLeftPidController{
      m_ClimberLP,
      0.0,
      0.0};

    double m_ClimberRMaxOutputValue = 1.000;               
    double m_ClimberRP              = 0.00015;  

    frc::PIDController m_ClimberRightPidController{
      m_ClimberRP,
      0.0,
      0.0};
  #endif


};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/
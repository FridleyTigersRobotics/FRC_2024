#pragma once

#include <Phoenix5.h>
#include <Constants.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

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
 private:
    ClimberState_t m_ClimberState {ClimberState_t::ClimberStop};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_leftClimberMotor { ConstantCrap::kLeftClimberMotor };
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_rightClimberMotor{ ConstantCrap::kRightClimberMotor };

    frc::Encoder m_motorEncoderL {2,3};
    frc::Encoder m_motorEncoderR {4,5};

    frc::DigitalInput m_rightLimitSwitch{ConstantCrap::kRightClimberStopDIO}; 
    frc::DigitalInput m_leftLimitSwitch {ConstantCrap::kLeftClimberStopDIO};
};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/
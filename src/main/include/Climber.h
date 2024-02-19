#pragma once

#include <Phoenix5.h>
#include <Constants.h>
#include <frc/DigitalInput.h>


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
 private:
    ClimberState_t m_ClimberState {ClimberState_t::ClimberStop};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_leftClimberMotor { ConstantCrap::kLeftClimberMotor };
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_rightClimberMotor{ ConstantCrap::kRightClimberMotor };



    frc::DigitalInput m_rightLimitSwitch{ConstantCrap::kRightClimberStopDIO}; 
    frc::DigitalInput m_leftLimitSwitch {ConstantCrap::kLeftClimberStopDIO};
};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/
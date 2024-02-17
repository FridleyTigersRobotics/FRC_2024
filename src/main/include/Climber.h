#pragma once

#include <Phoenix5.h>
typedef enum ClimberState_e
{
    ClimberDown,
    ClimberUp,
    ClimberStop
} ClimberState_t;

class Climber
{
 public:
    void initClimber();
    void updateClimber (/*Me when the me when... *Literally combusts* */);
    void ChangeClimberState( ClimberState_t ClimberState );

 private:
    ClimberState_t m_ClimberState {ClimberState_t::ClimberStop};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_leftClimberMotor { 23 };
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_rightClimberMotor{ 24 };
};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/
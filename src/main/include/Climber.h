
#include <ctre/Phoenix.h>
typedef enum ClimberState_e
{
    ClimberDown,
    ClimberUp,
    ClimberStop

} ClimberState_t;

class Climber
{
    void updateClimber (/*Me when the me when... *Literally combusts* */);
    void ChangeClimberState (/*Arises from ashes because epic pheonix*/);

ClimberState_t m_ClimberState {ClimberState_t::ClimberStop};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_leftClimberMotor { 4 };
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_rightClimberMotor{ 3 };
};
/*doalways: amogusdance
fofever: amogusdancebigfunny
never: amogusdancestopbefunny*/
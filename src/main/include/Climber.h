
#include <ctre/Phoenix.h>

class Climber
{
    void updateClimber (/*among us*/);

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_leftClimberMotor { 4 };
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  m_rightClimberMotor{ 3 };
};

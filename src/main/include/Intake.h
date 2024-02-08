#pragma once

#include <Phoenix5.h>
#include <rev/cansparkmax.h>
#include <Constants.h>
#include <frc/AnalogInput.h>
typedef enum intake_movement
{
    Intake_Intaking,
    Intake_Outtaking,
    Intake_Stopped,

} intake_movement_t;


class Intake{
 public:
    void ChangeIntakeState(intake_movement_t);
    intake_movement_t m_intake_movement;
    void updateIntake ();
rev::CANSparkMax m_IntakeMotor { ConstantCrap::kIntakeMotorcanID,rev::CANSparkLowLevel::MotorType::kBrushless };

//0=Ring is there :D
  frc::AnalogInput m_RingDetector{0};
   
    bool IsRingNotDetected() 
        {
         return m_RingDetector.GetValue() > 50;
        }
};

#include <Debug.h>
#include <Arm.h>
#include <rev/cansparkmax.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include <arms> *flexes, cutely*



#define WRIST_TEST_POSITIONS    ( 0 )
#define ARM_TEST_POSITIONS      ( 0 )

#define WRIST_DISTANCE_FIX_0 ( 1 )
#define WRIST_COLLIDING_FIX  ( 1 )

Arm::Arm()
{
    m_ArmMotorRight.SetInverted( false );
    m_ArmMotorLeft.SetInverted( true );
    m_WristMotor.SetInverted( true );
    m_ArmMotorLeftEncoder.SetPositionConversionFactor(  1.0 / ( 20.0 * ( 74.0 / 14.0 ) ) );
    m_ArmMotorRightEncoder.SetPositionConversionFactor( 1.0 / ( 20.0 * ( 74.0 / 14.0 ) ) );

    m_ArmMotorLeftEncoder.SetVelocityConversionFactor(  1.0 / ( 60 * 20.0 * ( 74.0 / 14.0 ) ) );
    m_ArmMotorRightEncoder.SetVelocityConversionFactor( 1.0 / ( 60 * 20.0 * ( 74.0 / 14.0 ) ) );


    m_pidControllerLeft.SetP(kP);
    m_pidControllerLeft.SetI(kI);
    m_pidControllerLeft.SetD(kD);
    m_pidControllerLeft.SetIZone(kIz);
    m_pidControllerLeft.SetFF(kFF);
    m_pidControllerLeft.SetOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerLeft.SetSmartMotionMaxVelocity(kMaxVel);
    m_pidControllerLeft.SetSmartMotionMinOutputVelocity(kMinVel);
    m_pidControllerLeft.SetSmartMotionMaxAccel(kMaxAcc);
    m_pidControllerLeft.SetSmartMotionAllowedClosedLoopError(kAllErr);


    m_pidControllerRight.SetP(kP);
    m_pidControllerRight.SetI(kI);
    m_pidControllerRight.SetD(kD);
    m_pidControllerRight.SetIZone(kIz);
    m_pidControllerRight.SetFF(kFF);
    m_pidControllerRight.SetOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerRight.SetSmartMotionMaxVelocity(kMaxVel);
    m_pidControllerRight.SetSmartMotionMinOutputVelocity(kMinVel);
    m_pidControllerRight.SetSmartMotionMaxAccel(kMaxAcc);
    m_pidControllerRight.SetSmartMotionAllowedClosedLoopError(kAllErr);

    // 44 / 18 sprockets
  #if WRIST_USE_MOTOR_ENCODER
    m_WristMotorEncoder.SetPositionConversionFactor( 1.0 / ( 25.0 * ( 44 / 18 ) ) );
    m_WristMotorEncoder.SetVelocityConversionFactor(  1.0 / ( 60 * 25.0 * ( 44 / 18 ) ) );
    m_WristPidController.SetP(kP2);
    m_WristPidController.SetI(kI2);
    m_WristPidController.SetD(kD2);
    m_WristPidController.SetIZone(kIz2);
    m_WristPidController.SetFF(kFF2);
    m_WristPidController.SetOutputRange(kMinOutput2, kMaxOutput2);
    m_WristPidController.SetSmartMotionMaxVelocity(kMaxVel2);
    m_WristPidController.SetSmartMotionMinOutputVelocity(kMinVel2);
    m_WristPidController.SetSmartMotionMaxAccel(kMaxAcc2);
    m_WristPidController.SetSmartMotionAllowedClosedLoopError(kAllErr2);
  #endif

#if 0
    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    frc::SmartDashboard::PutNumber("Max Velocity", kMaxVel);
    frc::SmartDashboard::PutNumber("Min Velocity", kMinVel);
    frc::SmartDashboard::PutNumber("Max Acceleration", kMaxAcc);
    frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", kAllErr);
    frc::SmartDashboard::PutNumber("Set Position", 0);
    frc::SmartDashboard::PutNumber("Set Velocity", 0);
    #endif     




 #if ARM_TEST_POSITIONS
    frc::SmartDashboard::PutNumber("Arm_Test_Ground", m_ArmGroundValue);
    frc::SmartDashboard::PutNumber("Arm_Test_Source", m_ArmSourceValue);
    frc::SmartDashboard::PutNumber("Arm_Test_Speaker", m_ArmSpeakerValue);
    frc::SmartDashboard::PutNumber("Arm_Test_Amp", m_ArmAmpValue);
    frc::SmartDashboard::PutNumber("Arm_Test_Trap", m_ArmTrapValue);
    frc::SmartDashboard::PutNumber("Arm_Test_MaxOutput", m_ArmMaxOutputValue);
    frc::SmartDashboard::PutNumber("Arm_Test_P", m_ArmP);
    frc::SmartDashboard::PutNumber("Arm_Test_MaxVel", m_ArmMaxVel);
    frc::SmartDashboard::PutNumber("Arm_Test_MaxAccel", m_ArmMaxAccel);
 #endif

 #if WRIST_TEST_POSITIONS
    frc::SmartDashboard::PutNumber("Wrist_Test_Ground",    m_WristGroundValue);
    frc::SmartDashboard::PutNumber("Wrist_Test_Source",    m_WristSourceValue);
    frc::SmartDashboard::PutNumber("Wrist_Test_Speaker",   m_WristSpeakerValue);
    frc::SmartDashboard::PutNumber("Wrist_Test_Amp",       m_WristAmpValue);
    frc::SmartDashboard::PutNumber("Wrist_Test_Trap",      m_WristTrapValue);
    frc::SmartDashboard::PutNumber("Wrist_Test_MaxOutput", m_WristMaxOutputValue);
    frc::SmartDashboard::PutNumber("Wrist_Test_P",         m_WristP);
    frc::SmartDashboard::PutNumber("Wrist_Test_MaxVel",    m_WristMaxVel);
    frc::SmartDashboard::PutNumber("Wrist_Test_MaxAccel",  m_WristMaxAccel);
 #endif
    
}

void Arm::initArm()
{
    m_ArmMotorRight.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_ArmMotorLeft.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_WristMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_ArmPosition     = arm_position_t::HOLD_START_POSITION;
    m_PrevArmPosition = arm_position_t::HOLD_START_POSITION;
    m_ArmMotorLeftEncoder.SetPosition(  m_ArmEncoder.GetAbsolutePosition() );
    m_ArmMotorRightEncoder.SetPosition( m_ArmEncoder.GetAbsolutePosition() );
    m_WristMotorEncoder.SetPosition( m_WristEncoder.GetAbsolutePosition() );



   #if WRIST_DISTANCE_FIX_0
    // Need to assume wrist is rotated outwards over the top
    if ( m_WristEncoder.GetAbsolutePosition() < 0.4 )
    {
        m_WristEncoderOffset = 1.0;
    }
    else
    {
        m_WristEncoderOffset = 0.0;
    }
   #endif
 
    m_startArmAngle   = m_ArmEncoder.GetAbsolutePosition();
    m_startWristAngle = getWristEncoderValue();


    m_WristPIDController.Reset( units::radian_t{m_startWristAngle} );
    
}


double Arm::getWristEncoderValue()
{
    return m_WristEncoderOffset + m_WristEncoder.GetDistance();
}


void Arm::disableArm()
{
    // Nice to be able to move the arm/wrist, but will slam down when disabled....
    m_ArmMotorRight.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    m_ArmMotorLeft.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    m_WristMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
}

void Arm::SetArmPosition (arm_position_t DesiredPosition)
{
    m_ArmPosition = DesiredPosition;
}

bool Arm::ArmHold()
{
    return m_ArmPosition == HOLD_START_POSITION;
}

bool Arm::ArmReadyForGroundIntake()
{
    return ( m_ArmPosition == GROUND_PICKUP &&
            getWristEncoderValue() > 0.9 );
}

bool Arm::ArmReadyForMoveForwardPreClimb()
{
    return ( m_ArmPosition == TRAP &&
            m_ArmEncoder.GetAbsolutePosition() < 0.2 );
}


units::meter_t Arm::ArmEndPosition()
{
    double armAngleFromStartingConfig = 2.0 * std::numbers::pi * ( m_ArmGroundValue - m_ArmEncoder.GetAbsolutePosition() );
    double armAngleFromLevel = double{kArmGroundAngle} + armAngleFromStartingConfig;
    double armEndPosition = cos( double{kArmGroundAngle} ) - cos( double{armAngleFromLevel} );

    frc::SmartDashboard::PutNumber( "Move_Angle",                double{180.0*armAngleFromLevel/std::numbers::pi} );

    return ( armEndPosition * kArmLength );
}

bool Arm::ArmReadyForShooting()
{   
    double wristEncoderVal = getWristEncoderValue();
    return ( m_ArmPosition   == SPEAKER &&
             wristEncoderVal == std::clamp( wristEncoderVal, -m_WristSpeakerValue, m_WristSpeakerValue ) );
}

void Arm::updateArm()
{
    double ArmAngle = 0;
    double WristAngle = getWristEncoderValue();
#if 0
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
    double minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
    double maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
    double allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP))   { m_pidControllerRight.SetP(p);m_pidControllerLeft.SetP(p); kP = p; }
    if((i != kI))   { m_pidControllerRight.SetI(i);m_pidControllerLeft.SetI(i); kI = i; }
    if((d != kD))   { m_pidControllerRight.SetD(d);m_pidControllerLeft.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidControllerRight.SetIZone(iz);m_pidControllerLeft.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidControllerRight.SetFF(ff);m_pidControllerLeft.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { m_pidControllerRight.SetOutputRange(min, max);m_pidControllerLeft.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
    if((maxV != kMaxVel)) { m_pidControllerRight.SetSmartMotionMaxVelocity(maxV);m_pidControllerLeft.SetSmartMotionMaxVelocity(maxV); kMaxVel = maxV; }
    if((minV != kMinVel)) { m_pidControllerRight.SetSmartMotionMinOutputVelocity(minV);m_pidControllerLeft.SetSmartMotionMinOutputVelocity(minV); kMinVel = minV; }
    if((maxA != kMaxAcc)) { m_pidControllerRight.SetSmartMotionMaxAccel(maxA);m_pidControllerLeft.SetSmartMotionMaxAccel(maxA); kMaxAcc = maxA; }
    if((allE != kAllErr)) { m_pidControllerRight.SetSmartMotionAllowedClosedLoopError(allE); m_pidControllerLeft.SetSmartMotionAllowedClosedLoopError(allE); allE = kAllErr; }
#endif



 #if ARM_TEST_POSITIONS
    double temp_m_ArmGroundValue = frc::SmartDashboard::GetNumber("Arm_Test_Ground", m_ArmGroundValue);
    double temp_m_ArmSourceValue = frc::SmartDashboard::GetNumber("Arm_Test_Source", m_ArmSourceValue);
    double temp_m_ArmSpeakerValue = frc::SmartDashboard::GetNumber("Arm_Test_Speaker", m_ArmSpeakerValue);
    double temp_m_ArmAmpValue = frc::SmartDashboard::GetNumber("Arm_Test_Amp", m_ArmAmpValue);
    double temp_m_ArmTrapValue = frc::SmartDashboard::GetNumber("Arm_Test_Trap", m_ArmTrapValue);
    double temp_m_ArmMaxOutputValue = frc::SmartDashboard::GetNumber("Arm_Test_MaxOutput", m_ArmMaxOutputValue);
    double temp_m_ArmP = frc::SmartDashboard::GetNumber("Arm_Test_P", m_ArmP);
    double temp_m_ArmMaxVel = frc::SmartDashboard::GetNumber("Arm_Test_MaxVel", m_ArmMaxVel);
    double temp_m_ArmMaxAccel = frc::SmartDashboard::GetNumber("Arm_Test_MaxAccel", m_ArmMaxAccel);

    if ( temp_m_ArmGroundValue != m_ArmGroundValue) m_ArmGroundValue = temp_m_ArmGroundValue;
    if ( temp_m_ArmSourceValue != m_ArmSourceValue) m_ArmSourceValue = temp_m_ArmSourceValue;
    if ( temp_m_ArmSpeakerValue != m_ArmSpeakerValue) m_ArmSpeakerValue = temp_m_ArmSpeakerValue;
    if ( temp_m_ArmAmpValue != m_ArmAmpValue) m_ArmAmpValue = temp_m_ArmAmpValue;
    if ( temp_m_ArmTrapValue != m_ArmTrapValue) m_ArmTrapValue = temp_m_ArmTrapValue;
    if ( temp_m_ArmMaxOutputValue != m_ArmMaxOutputValue) {m_ArmMaxOutputValue = temp_m_ArmMaxOutputValue; m_pidControllerLeft.SetOutputRange(m_ArmMaxOutputValue, -m_ArmMaxOutputValue); m_pidControllerRight.SetOutputRange(m_ArmMaxOutputValue, -m_ArmMaxOutputValue);};
    if ( temp_m_ArmP != m_ArmP) {m_ArmP = temp_m_ArmP; m_pidControllerLeft.SetP(m_ArmP); m_pidControllerRight.SetP(m_ArmP);};
    if ( temp_m_ArmMaxVel != m_ArmMaxVel) {m_ArmMaxVel =temp_m_ArmMaxVel; m_pidControllerLeft.SetSmartMotionMaxVelocity(m_ArmMaxVel); m_pidControllerRight.SetSmartMotionMaxVelocity(m_ArmMaxVel);};
    if ( temp_m_ArmMaxAccel != m_ArmMaxAccel) {m_ArmMaxAccel = temp_m_ArmMaxAccel; m_pidControllerLeft.SetSmartMotionMaxAccel(m_ArmMaxAccel); m_pidControllerRight.SetSmartMotionMaxAccel(m_ArmMaxAccel);};
 #endif


 #if WRIST_TEST_POSITIONS
    double temp_m_WristGroundValue = frc::SmartDashboard::GetNumber("Wrist_Test_Ground", m_WristGroundValue);
    double temp_m_WristSourceValue = frc::SmartDashboard::GetNumber("Wrist_Test_Source", m_WristSourceValue);
    double temp_m_WristSpeakerValue = frc::SmartDashboard::GetNumber("Wrist_Test_Speaker", m_WristSpeakerValue);
    double temp_m_WristAmpValue = frc::SmartDashboard::GetNumber("Wrist_Test_Amp", m_WristAmpValue);
    double temp_m_WristTrapValue = frc::SmartDashboard::GetNumber("Wrist_Test_Trap", m_WristTrapValue);
    double temp_m_WristMaxOutputValue = frc::SmartDashboard::GetNumber("Wrist_Test_MaxOutput", m_WristMaxOutputValue);
    double temp_m_WristP = frc::SmartDashboard::GetNumber("Wrist_Test_P", m_WristP);
    double temp_m_WristMaxVel = frc::SmartDashboard::GetNumber("Wrist_Test_MaxVel", m_WristMaxVel);
    double temp_m_WristMaxAccel = frc::SmartDashboard::GetNumber("Wrist_Test_MaxAccel", m_WristMaxAccel);

    if ( temp_m_WristGroundValue != m_WristGroundValue) m_WristGroundValue = temp_m_WristGroundValue;
    if ( temp_m_WristSourceValue != m_WristSourceValue) m_WristSourceValue = temp_m_WristSourceValue;
    if ( temp_m_WristSpeakerValue != m_WristSpeakerValue) m_WristSpeakerValue = temp_m_WristSpeakerValue;
    if ( temp_m_WristAmpValue != m_WristAmpValue) m_WristAmpValue = temp_m_WristAmpValue;
    if ( temp_m_WristTrapValue != m_WristTrapValue) m_WristTrapValue = temp_m_WristTrapValue;
    if ( temp_m_WristMaxOutputValue != m_WristMaxOutputValue) m_WristMaxOutputValue = temp_m_WristMaxOutputValue;
    if ( temp_m_WristP != m_WristP) {m_WristP = temp_m_WristP; m_WristPIDController.SetP(m_ArmP);};

    if ( temp_m_WristMaxVel != m_WristMaxVel || 
         temp_m_WristMaxAccel != m_WristMaxAccel) 
    {
        m_WristMaxVel   = temp_m_WristMaxVel;
        m_WristMaxAccel = temp_m_WristMaxAccel;
        m_WristPIDController.SetConstraints({units::radians_per_second_t{m_WristMaxVel}, units::radians_per_second_squared_t{m_WristMaxAccel}});
    }
 #endif

    if ( m_ArmPosition != m_PrevArmPosition )
    {
        m_startArmAngle   = m_ArmEncoder.GetAbsolutePosition();
        m_startWristAngle = getWristEncoderValue(); 
    }

    m_PrevArmPosition = m_ArmPosition;

    switch (m_ArmPosition)
    {   
        case (HOLD_START_POSITION):
        {
            ArmAngle   = m_startArmAngle;
            WristAngle = getWristEncoderValue();
            break;
        }
        // TODO : Determine all arm and wrist positions
        case (GROUND_PICKUP):
        {
            ArmAngle   = m_ArmGroundValue;
            WristAngle = m_WristGroundValue;
            break;
        }
        case (SOURCE):
        {
            ArmAngle = m_ArmSourceValue;
           #if WRIST_COLLIDING_FIX
            // Only move the wrist if it has enough clearance to move.
            /*if ( m_ArmEncoder.GetAbsolutePosition() > 0.45 )
            {
                WristAngle = m_startWristAngle;
            }
            else*/
           #endif
            {
                WristAngle = m_WristSourceValue;
            }
            break;
        }
        case (SPEAKER):
        {
            ArmAngle   = m_ArmSpeakerValue;
            WristAngle = m_WristSpeakerValue;
            break;
        }
        case (AMP):
        {
            ArmAngle = m_ArmAmpValue;
           #if WRIST_COLLIDING_FIX
            // Only move the wrist if it has enough clearance to move.
            /*if ( m_ArmEncoder.GetAbsolutePosition() > 0.45 )
            {
                WristAngle = m_startWristAngle;
            }
            else*/
           #endif
            {
                WristAngle = m_WristAmpValue;
            }
            break;
        }
        case (TRAP):
        {
            ArmAngle   = m_ArmTrapValue;
            WristAngle = m_WristTrapValue;
            break;
        }
    }
    
    //PIDdly thing
    // const auto ArmControlOutput = m_ArmPIDController.Calculate(
    //     units::radian_t{m_ArmEncoder.GetDistance()}, units::radian_t{ArmAngle});

    m_ArmAngle   = ArmAngle;
    m_WristAngle = WristAngle;

   #if DBG_DISABLE_ARM_MOTORS
    m_ArmMotorLeft.Set( 0 );
    m_ArmMotorRight.Set( 0 );
   #else
    m_pidControllerLeft.SetReference(ArmAngle, rev::CANSparkMax::ControlType::kSmartMotion);
    m_pidControllerRight.SetReference(ArmAngle, rev::CANSparkMax::ControlType::kSmartMotion);
   #endif

   #if DBG_DISABLE_WRIST_MOTORS
    m_WristMotor.Set( 0 );
   #elif WRIST_USE_MOTOR_ENCODER
    m_WristPidController.SetReference(WristAngle, rev::CANSparkMax::ControlType::kSmartMotion);
   #else

    m_WristControlOutput = m_WristPIDController.Calculate(
        units::radian_t{getWristEncoderValue()}, units::radian_t{WristAngle});

   if ( m_ArmPosition != HOLD_START_POSITION )
   {
    //double const feedForward = 0.0; // TODO : Do we need feed forward based on wrist angle relative to the ground?
    m_WristMotor.Set( std::clamp( m_WristControlOutput, -m_WristMaxOutputValue, m_WristMaxOutputValue ) );
   }
   else
   {
    m_WristMotor.Set( 0.0 );
   }

   #endif
}

void Arm::UpdateSmartDashboardData()
{
#if 1
    //frc::SmartDashboard::PutNumber("Arm_ControlOutputL",   m_ArmMotorLeft.GetAppliedOutput());
    //frc::SmartDashboard::PutNumber("Arm_ControlOutputR",   m_ArmMotorRight.GetAppliedOutput());

    frc::SmartDashboard::PutNumber("Arm_Angle",           m_ArmAngle);
    //frc::SmartDashboard::PutNumber("Arm_Position",        m_ArmPosition);
    frc::SmartDashboard::PutNumber("Arm_Encoder_Dist",    m_ArmEncoder.GetDistance());//Ground pickup:0.5099 Top Position 0.1844
    frc::SmartDashboard::PutNumber("Arm_Encoder_AbsPos",  m_ArmEncoder.GetAbsolutePosition());//Ground pickup:0.5099 Top Position 0.1844
    
    frc::SmartDashboard::PutNumber("Arm_NeoPositionL",     m_ArmMotorLeftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Arm_NeoPositionR",     m_ArmMotorRightEncoder.GetPosition());

    //frc::SmartDashboard::PutNumber("Arm_NeoVelocityL",     m_ArmMotorLeftEncoder.GetVelocity());
    //frc::SmartDashboard::PutNumber("Arm_NeoVelocityR",     m_ArmMotorRightEncoder.GetVelocity());
#endif
    frc::SmartDashboard::PutNumber("Wrist_m_ArmPosition",    m_ArmPosition);    
    frc::SmartDashboard::PutNumber("Wrist_Encoder_Dist",     getWristEncoderValue());
    frc::SmartDashboard::PutNumber("Wrist_Encoder_AbsPos",   m_WristEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("Wrist_MotorEncoder_Pos", m_WristMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Wrist_ControlOutput",    m_WristControlOutput);
    frc::SmartDashboard::PutNumber("Wrist_AppliedOutput",    m_WristMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Wrist_NeoVelocity",      m_WristMotorEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Wrist_WristAngle",      m_WristAngle);
}





void Arm::armManualControl( double speed )
{
    m_ArmMotorRight.Set(0.4 * speed);
    m_ArmMotorLeft.Set(0.4 * speed);
}


void Arm::wristManualControl( double speed )
{
    m_WristMotor.Set(0.2 * speed);
}


//Change to work for wrist???

//PID? PIN? Bowling? Wii Sports Bowling???

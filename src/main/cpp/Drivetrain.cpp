// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/printf.h>

void Drivetrain::Drive(
  units::meters_per_second_t xSpeed,
  units::meters_per_second_t ySpeed,
  units::radians_per_second_t rot, 
  bool fieldRelative,
  units::second_t period
) 
{
  frc::ChassisSpeeds FieldRelativeChassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, frc::Rotation2d{units::degree_t {m_imu.GetYaw()}});
  frc::ChassisSpeeds RobotRelativeChassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, rot};
  frc::ChassisSpeeds ChassisSpeeds = frc::ChassisSpeeds::Discretize( RobotRelativeChassisSpeeds, period );
  UpdateOdometry();
  auto states = m_kinematics.ToSwerveModuleStates( ChassisSpeeds );

  m_kinematics.DesaturateWheelSpeeds( &states, kMaxSpeed );

  auto [fl, fr, bl, br] = states;

  // Check if the wheels don't have a drive velocity to maintain the current wheel orientation.
  bool hasVelocity = fl.speed != 0_mps || fr.speed != 0_mps || bl.speed != 0_mps || br.speed != 0_mps;

  if ( !hasVelocity )
  {
    fl.angle = m_frontLeft.GetState().angle;
    fr.angle = m_frontRight.GetState().angle;
    bl.angle = m_backLeft.GetState().angle;
    br.angle = m_backRight.GetState().angle;
  }

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateSmartDashboardData()
{
  frc::SmartDashboard::PutNumber( "Drive_X", double{m_odometry.GetPose().X()} );
  frc::SmartDashboard::PutNumber( "Drive_y", double{m_odometry.GetPose().Y()} );
  frc::SmartDashboard::PutNumber( "Drive_Rot", double{m_odometry.GetPose().Rotation().Degrees()} );
}


void Drivetrain::UpdateOdometry() {
  m_odometry.Update(frc::Rotation2d{units::degree_t {m_imu.GetYaw()}},
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

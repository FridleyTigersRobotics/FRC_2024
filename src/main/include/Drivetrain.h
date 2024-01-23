// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "SwerveModule.h"
#include "frc/geometry/Rotation2d.h"

/**
 * Represents a swerve drive style drivetrain.
 */

//32^2=a^2+b^s    512

class Drivetrain {
 public:
  Drivetrain() { /*m_gyro.Reset();*/ }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      std::numbers::pi};  // 1/2 rotation per second

 private:


   //-----------|Front|------------
  //  16----------------------12
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  14----------------------10
  //------------|back|-------------
  /// 32 in diagonal
  // 22.627417
  //11.3137085
  frc::Translation2d m_frontLeftLocation {+0.287_m, +0.287_m};
  frc::Translation2d m_frontRightLocation{+0.287_m, -0.287_m};
  frc::Translation2d m_backLeftLocation  {-0.287_m, +0.287_m};
  frc::Translation2d m_backRightLocation {-0.287_m, -0.287_m};

//1.230863 Drive motor #10
//0.909437 Drive motor #12
//0.255626 Drive motor #14
//4.980153 Drive motor #16

  SwerveModule m_frontLeft {10, 11, 0, (1.230863/(2*std::numbers::pi))};
  SwerveModule m_frontRight{12, 13, 1, 0.909437/(2*std::numbers::pi)};
  SwerveModule m_backLeft  {14, 15, 2, 0.255626/(2*std::numbers::pi)};
  SwerveModule m_backRight {16, 17, 3, 4.980153/(2*std::numbers::pi)};

  //frc::AnalogGyro m_gyro{4};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      frc::Rotation2d{units::radian_t {0}},//m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};

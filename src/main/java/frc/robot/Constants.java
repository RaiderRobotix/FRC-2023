// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public interface Constants {
  public final int xboxControllerPort = 4;

  // Neo Spark Maxes constants
  public final double kGearRatio = 6.75;
  public final int kMaxRPM = 6380;
  public final double kTireCircumference = 1.5;
  public final double kUnitsPerRevoltion = kGearRatio * kTireCircumference;

  public final double rightDeadband = 0.15;
  public final double leftDeadband = 0.15;
  public final double rightTriggerThreshold = .70;
  public final double leftTriggerThreshold = .70;

  // Measurements in metres
  public final double width = Units.inchesToMeters(24);
  public final double length = Units.inchesToMeters(32);

  public final double frontLeftLocationX = 1.0;
  public final double frontLeftLocationY = 1.0;

  public final double frontRightLocationX = 1.0;
  public final double frontRightLocationY = 1.0;

  public final double backLeftLocationX = 1.0;
  public final double backLeftLocationY = 1.0;

  public final double backRightLocationX = 1.0;
  public final double backRightLocationY = 1.0;

  public final SwerveDriveKinematics kDriveKinematics = 
  new SwerveDriveKinematics(
      new Translation2d(width / 2, length / 2),
      new Translation2d(width / 2, -length / 2),
      new Translation2d(-width / 2, length / 2),
      new Translation2d(-width / 2, -length / 2)
  );

  public final double kPhysicalDriveMaxSpeed = 1;
  public final double maxAttainableSpeed = 18.0;
  public final double kPhysicalSteerMaxSpeed = 5;
  public final double rotateSpeed = 0.5;
  public final double kMaxAccelerationMetersPerSecondSquared = 0.5;
  public final double turboSpeed = 1;
  public final double normalSpeed = 0.6;
  public final double slowSpeed = 0.2;

  public final int frontLeftDriveID = 20;
  public final int frontLeftSteerID = 10;
  public final int frontLeftEncoderID = 0;
  public final double frontLeftEncoderOffset = -18;

  public final int frontRightDriveID = 21;
  public final int frontRightSteerID = 11;
  public final int frontRightEncoderID = 1;
  public final double frontRightEncoderOffset = -10.61;

  public final int backLeftDriveID = 22;
  public final int backLeftSteerID = 12;
  public final int backLeftEncoderID = 2;
  public final double backleftEncoderOffset = 280.9;

  public final int backRightDriveID = 23;
  public final int backRightSteerID = 13;
  public final int backRightEncoderID = 3;
  public final double backRightEncoderOffset = 2.59;

  // Set kp, ki, and kd to zero.

  // Increase kp until the output starts to oscillate around the setpoint.

  // Increase kd as much as possible without introducing jittering in the system
  // response.

  // Increase ki

  public final double angleKp = 0.0030; // Speed Default 0.01100
  public final double angleKi = 0.0000; // Amount to react Default 0.00001
  public final double angleKd = 0.0000; // Dampens system Default 0.00037

  public final double driveKp = 500;
  public final double driveKi = 0.055;
  public final double driveKd = 0.00;

    
  //PID values for commands
  //PID values for robot angle/heading
  public final double robotangleKi = 0.00100; // Amount to react Default 0.00001
  public final double robotangleKp = 0.00000; // Speed Default 0.01100
  public final double robotangleKd = 0.00000; // Dampens system Default 0.00037

  //PID values for robot drive distance
  public final double robotDriveDistanceKi = 0.2000;
  public final double robotDriveDistanceKp = 0.0000;
  public final double robotDriveDistanceKd = 0.0000;

  //PID values for thete controller
  public final double thetaControllerKp = 0.5000;
  public final double thetaControllerKi = 0.0000;
  public final double thetaControllerKd = 0.0000;

  //PID values for position controller
  //X value
  public final double xControllerKp = 0.5000;
  public final double xControllerKi = 0.1100; //Dont use
  public final double xControllerKd = 0.0000; //Dont
  //Y Value
  public final double yControllerKp = 0.5000;
  public final double yControllerKi = 0.0000; //Dont use
  public final double yControllerKd = 0.0000; //Dont use

  //PID values for robot balance command
  public final double balanceKp = 0.0000;
  public final double balanceKi = 0.0000;
  public final double balanceKd = 0.0000;

  public static final TrapezoidProfile.Constraints kThetaControllerConstraint = new TrapezoidProfile.Constraints(
          kPhysicalDriveMaxSpeed, kPhysicalSteerMaxSpeed);

}

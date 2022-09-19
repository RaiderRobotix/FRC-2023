// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveWheelController extends SubsystemBase implements drivebaseConstants {
  private static SwerveWheelController instance;

  private SwerveWheel frontLeftModule;
  private SwerveWheel frontRightModule;
  private SwerveWheel backLeftModule;
  private SwerveWheel backRightModule;

  /** Creates a new drivebase. */
  public SwerveWheelController() {
    // Location of modules relative to the centre of the robot
    Translation2d frontLeftLocation = new Translation2d(frontLeftLocationX, frontLeftLocationY);
    Translation2d frontRightLocation = new Translation2d(frontRightLocationX, frontRightLocationY);
    Translation2d backLeftLocation = new Translation2d(backLeftLocationX, backLeftLocationY);
    Translation2d backRightLocation = new Translation2d(backRightLocationX, backRightLocationY);

    frontLeftModule = new SwerveWheel(driveID, steerID, encoderID, name);
    frontRightModule = new SwerveWheel(driveID, steerID, encoderID, name);
    backLeftModule = new SwerveWheel(driveID, steerID, encoderID, name);
    backRightModule = new SwerveWheel(driveID, steerID, encoderID, name);

    // System.out.println(frontLeftLocation.getNorm());

    // Creates kinematics object using the above module location
    // TODO fill out missing variables
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    ChassisSpeeds speeds = new ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, currentHeading);

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        new Rotation2d(frontLeftModule.getSteerAngle()));
    var frontRightOptimized = SwerveModuleState.optimize(frontLeft,
        new Rotation2d(frontRightModule.getSteerAngle()));
    var backLeftOptimized = SwerveModuleState.optimize(frontLeft,
        new Rotation2d(backLeftModule.getSteerAngle()));
    var backRightOptimized = SwerveModuleState.optimize(frontLeft,
        new Rotation2d(backRightModule.getSteerAngle()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static SwerveWheelController getInstance() {
    if (instance == null) {
      instance = new SwerveWheelController();
      // instance.setDefaultCommand(new TeleopDrive());
    }

    return instance;
  }
}

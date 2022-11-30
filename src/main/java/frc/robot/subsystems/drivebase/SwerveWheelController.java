// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.Gyro;

public class SwerveWheelController extends SubsystemBase implements drivebaseConstants {

  private ChassisSpeeds speeds;

  private SwerveWheel frontLeftModule;
  private SwerveWheel frontRightModule;
  private SwerveWheel backLeftModule;
  private SwerveWheel backRightModule;

  private SwerveDriveKinematics kinematics;

  private PIDController angleController = new PIDController(angleKd, angleKi, angleKd);

  /** Creates a new drivebase. */
  public SwerveWheelController() {
    // Location of modules relative to the centre of the robot
    Translation2d frontLeftLocation = new Translation2d(frontLeftLocationX, frontLeftLocationY);
    Translation2d frontRightLocation = new Translation2d(frontRightLocationX, frontRightLocationY);
    Translation2d backLeftLocation = new Translation2d(backLeftLocationX, backLeftLocationY);
    Translation2d backRightLocation = new Translation2d(backRightLocationX, backRightLocationY);

    frontLeftModule = new SwerveWheel(frontLeftDriveID, frontLeftSteerID, frontLeftSteerEncoderID, "Front Left");
    frontRightModule = new SwerveWheel(frontLeftDriveID, frontRightSteerID, frontRightSteerEncoderID, "Front Right");
    backLeftModule = new SwerveWheel(backLeftDriveID, backLeftSteerID, backLeftSteerEncoderID, "Back Left");
    backRightModule = new SwerveWheel(backRightDriveID, backRightSteerID, backRightSteerEncoderID, "Back Right");

    // System.out.println(frontLeftLocation.getNorm());

    // Creates kinematics object using the above module location
    // TODO fill out missing variables
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    this.speeds = new ChassisSpeeds();

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    angleController.enableContinuousInput(0, 360);

  }

  public void setSpeed(double x, double y, double delta) {
    this.speeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(this.speeds);
    setState(moduleStates);
  }

  public void setState(SwerveModuleState[] moduleStates) {
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

  public void setHeading(double angle) {
    this.speeds.fromFieldRelativeSpeeds(getXSpeed(), getYSpeed(),
        angleController.calculate(getHeading(), angle),
        getRotation2d());
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    frontLeftModule.setDesiredAngle(moduleStates[0]);
    frontRightModule.setDesiredAngle(moduleStates[1]);
    backLeftModule.setDesiredAngle(moduleStates[2]);
    backRightModule.setDesiredAngle(moduleStates[3]);
  }

  public double getXSpeed() {
    return this.speeds.vxMetersPerSecond;
  }

  public double getYSpeed() {
    return this.speeds.vyMetersPerSecond;
  }

  public double getHeading() {
    return Gyro.gyro().getAngle();
  }

  public Rotation2d getRotation2d() {
    new Rotation2d();
    return Rotation2d.fromDegrees(Gyro.gyro().getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

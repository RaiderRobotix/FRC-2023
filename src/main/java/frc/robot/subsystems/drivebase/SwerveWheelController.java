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
import frc.robot.commands.drive;
import frc.robot.subsystems.Gyro;

public class SwerveWheelController extends SubsystemBase implements drivebaseConstants {

  private ChassisSpeeds speeds;

  private SwerveWheel frontLeftModule;
  private SwerveWheel frontRightModule;
  private SwerveWheel backLeftModule;
  private SwerveWheel backRightModule;

  Translation2d frontLeftLocation;
  Translation2d frontRightLocation;
  Translation2d backLeftLocation;
  Translation2d backRightLocation;

  private SwerveDriveKinematics kinematics;

  private PIDController angleController = new PIDController(angleKd, angleKi, angleKd);

  /** Creates a new drivebase. */
  public SwerveWheelController() {
    // Location of modules relative to the centre of the robot
    this.frontLeftLocation = new Translation2d(frontLeftLocationX, frontLeftLocationY);
    this.frontRightLocation = new Translation2d(frontRightLocationX, frontRightLocationY);
    this.backLeftLocation = new Translation2d(backLeftLocationX, backLeftLocationY);
    this.backRightLocation = new Translation2d(backRightLocationX, backRightLocationY);

    this.frontLeftModule = new SwerveWheel(frontLeftDriveID, frontLeftSteerID, frontLeftEncoderID, "Front Left");
    this.frontRightModule = new SwerveWheel(frontLeftDriveID, frontRightSteerID, frontRightEncoderID,
        "Front Right");
    this.backLeftModule = new SwerveWheel(backLeftDriveID, backLeftSteerID, backLeftEncoderID, "Back Left");
    this.backRightModule = new SwerveWheel(backRightDriveID, backRightSteerID, backRightEncoderID, "Back Right");

    // Creates kinematics object using the above module location
    // TODO fill out missing variables
    this.kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    angleController.enableContinuousInput(0, 360);

  }

  public void setSpeed(double x, double y, double delta) {
    this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(kPhysicalDriveMaxSpeed * x,
        kPhysicalDriveMaxSpeed * y,
        kPhysicalDriveMaxSpeed * delta, getRotation2d());

    System.out.println(speeds + " " + x + " " + y + " " + Math.toDegrees(delta));
    // System.out.println(this.kinematics);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    setState(moduleStates);
  }

  public void setState(SwerveModuleState[] moduleStates) {
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

  public void setHeading(double angle) {
    this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(getXSpeed(), getYSpeed(),
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

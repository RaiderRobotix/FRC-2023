// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private SwerveDriveOdometry odometry;

  private PIDController angleController = new PIDController(angleKd, angleKi,
      angleKd);

  /** Creates a new drivebase. */
  public SwerveWheelController() {

    // Location of modules relative to the centre of the robot
    this.frontLeftLocation = new Translation2d(frontLeftLocationX, frontLeftLocationY);
    this.frontRightLocation = new Translation2d(frontRightLocationX, frontRightLocationY);
    this.backLeftLocation = new Translation2d(backLeftLocationX, backLeftLocationY);
    this.backRightLocation = new Translation2d(backRightLocationX, backRightLocationY);

    this.frontLeftModule = new SwerveWheel(frontLeftDriveID, frontLeftSteerID, frontLeftEncoderID, "Front Left");
    this.frontRightModule = new SwerveWheel(frontRightDriveID, frontRightSteerID, frontRightEncoderID,
        "Front Right");
    this.backLeftModule = new SwerveWheel(backLeftDriveID, backLeftSteerID, backLeftEncoderID, "Back Left");
    this.backRightModule = new SwerveWheel(backRightDriveID, backRightSteerID, backRightEncoderID, "Back Right");

    // Creates kinematics object using the above module location
    // TODO fill out missing variables
    this.kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    this.odometry = new SwerveDriveOdometry(kinematics, getRotation2d());

    angleController.enableContinuousInput(0, 360);
    resetMotors();
  }

  public void resetMotors() {
    // frontLeftModule.setSteerAngle(90);
    frontLeftModule.resetMotors();
    frontRightModule.resetMotors();
    backLeftModule.resetMotors();
    backRightModule.resetMotors();
  }

  public void setSpeed(double x, double y, double delta) {
    this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());

    // System.out.println(this.kinematics);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    setState(moduleStates);
    // System.out.println(speeds + " " + speeds.vxMetersPerSecond + " " +
    // speeds.vyMetersPerSecond + " " + Math.toDegrees(speeds.omegaRadiansPerSecond)
    // + " Gyro: " + getHeading());
  }

  public void setState(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kPhysicalDriveMaxSpeed);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kPhysicalDriveMaxSpeed);
    frontLeftModule.setDesiredAngle(moduleStates[0]);
    frontRightModule.setDesiredAngle(moduleStates[1]);
    backLeftModule.setDesiredAngle(moduleStates[2]);
    backRightModule.setDesiredAngle(moduleStates[3]);
  }

  // Sets the angle of all the wheel to angle
  public void setAngle(double angle) {
    frontLeftModule.setSteerAngle(angle);
    frontRightModule.setSteerAngle(angle);
    backLeftModule.setSteerAngle(angle);
    backRightModule.setSteerAngle(angle);
  }

  public double getXSpeed() {
    if (this.speeds != null) {
      return this.speeds.vxMetersPerSecond;
    } else {
      return 0;
    }
  }

  public double getYSpeed() {
    if (this.speeds != null) {
      return this.speeds.vyMetersPerSecond;
    } else {
      return 0;
    }
  }

  public double getHeading() {
    return Gyro.gyro().getCompassHeading();
  }

  public Rotation2d getRotation2d() {
    new Rotation2d();
    if (Gyro.gyro() != null) {
      return Rotation2d.fromDegrees(Gyro.gyro().getCompassHeading());
    } else {
      return Rotation2d.fromDegrees(90);
    }
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(),
        backRightModule.getState());

    SmartDashboard.putNumber("X Speed", getXSpeed());
    SmartDashboard.putNumber("Y Speed", getYSpeed());

    SmartDashboard.updateValues();
    // This method will be called once per scheduler run
  }
}

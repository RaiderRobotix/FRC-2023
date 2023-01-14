// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants;
import frc.robot.commands.drive;
import frc.robot.subsystems.Gyro;

public class SwerveWheelController extends SubsystemBase implements drivebaseConstants, Constants {

  private ChassisSpeeds speeds;

  // private Gyro gyro = new Gyro();

  private static SwerveWheel frontLeftModule;
  private static SwerveWheel frontRightModule;
  private static SwerveWheel backLeftModule;
  private static SwerveWheel backRightModule;
  private static boolean fieldCentric = true;
  private static boolean coast = false;

  Translation2d frontLeftLocation;
  Translation2d frontRightLocation;
  Translation2d backLeftLocation;
  Translation2d backRightLocation;

  public SwerveDriveKinematics kinematics;

  private static SwerveWheel[] modules = new SwerveWheel[4];

  private SwerveDriveOdometry odometry;

  private PIDController angleController = new PIDController(robotangleKd, robotangleKi,
      robotangleKd);

  /** Creates a new drivebase. */
  public SwerveWheelController() {

    // Location of modules relative to the centre of the robot
    this.frontLeftLocation = new Translation2d(width / 2, length / 2);
    this.frontRightLocation = new Translation2d(width / 2, -length / 2);
    this.backLeftLocation = new Translation2d(-width / 2, length / 2);
    this.backRightLocation = new Translation2d(-width / 2, -length / 2);

    this.frontLeftModule = new SwerveWheel(frontLeftDriveID, frontLeftSteerID, frontLeftEncoderID, "Front Left");
    this.frontRightModule = new SwerveWheel(frontRightDriveID, frontRightSteerID, frontRightEncoderID,
        "Front Right");
    this.backLeftModule = new SwerveWheel(backLeftDriveID, backLeftSteerID, backLeftEncoderID, "Back Left");
    this.backRightModule = new SwerveWheel(backRightDriveID, backRightSteerID, backRightEncoderID, "Back Right");

    modules[0] = this.frontLeftModule;
    modules[1] = this.frontRightModule;
    modules[2] = this.backLeftModule;
    modules[3] = this.backRightModule;
    
    // Creates kinematics object using the above module location
    // TODO fill out missing variables
    this.kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    this.odometry = new SwerveDriveOdometry(kinematics, getRotation2d());

    angleController.enableContinuousInput(0, 360);
    resetMotors();
    resetEncoders();
  }

  public static void resetMotors() {
    // frontLeftModule.setSteerAngle(90);
    frontLeftModule.resetMotors();
    frontRightModule.resetMotors();
    backLeftModule.resetMotors();
    backRightModule.resetMotors();
  }

  public static void toggleField() {
    if (fieldCentric) {
      fieldCentric = false;
    } else {
      fieldCentric = true;
    }
  }

  public static void resetEncoders(){
    for(SwerveWheel module : modules){
      module.resetEncoder();
    }
  }

  public static void stopMotors() {
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getRotation2d());
  }

  public static void toggleCoast() {
    if (coast) {
      coast = false;
    } else {
      coast = true;
    }
    frontLeftModule.setNeutralMode(coast);
    frontRightModule.setNeutralMode(coast);
    backLeftModule.setNeutralMode(coast);
    backRightModule.setNeutralMode(coast);
  }

  public void setSpeed(double x, double y, double delta, double maxSpeed) {
    SmartDashboard.putNumber("DesiredXSpeed", x);
    SmartDashboard.putNumber("DesiredYSpeed", y);
    if (true) {
      this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());
      // this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, new
      // Rotation2d().fromDegrees(45));
      // System.out.println(rotate.getDegrees());
    } else {
      this.speeds = new ChassisSpeeds(x, y, delta);

    }
    // System.out.println(this.kinematics);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    setState(moduleStates, maxSpeed);
    // System.out.println(speeds + " " + speeds.vxMetersPerSecond + " " +
    // speeds.vyMetersPerSecond + " " + Math.toDegrees(speeds.omegaRadiansPerSecond)
    // + " Gyro: " + getHeading());
  }

  public void setState(SwerveModuleState[] moduleStates, double maxSpeed) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

  public void setHeading(double angle, double maxSpeed) {
    setSpeed(0, 0, angleController.calculate(Gyro.getHeading(), angle), maxSpeed);
    // System.out.println(angle + " " + Gyro.getHeading());
    // // angle += 180;
    // boolean reverse;
    // if (a
    // ngle - Gyro.getHeading() > 360 - angle) {
    // reverse = true;
    // } else {
    // reverse = false;
    // }

    // System.out.println("Angle: " + angle + " Yaw " + Gyro.getHeading());
    // // Make Reverse dependent on which value it is closer too
    // while ((Gyro.getHeading() - angle) > 1) {
    // if (!reverse) {
    // setSpeed(0, 0, drivebaseConstants.rotateSpeed,
    // drivebaseConstants.rotateSpeed);
    // } else {
    // setSpeed(0, 0, -drivebaseConstants.rotateSpeed,
    // drivebaseConstants.rotateSpeed);
    // }
    // SmartDashboard.updateValues();
    // }

  }

  // Sets the angle of all the wheel to angle
  public static void setAngle(double angle) {
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

  public void setSteerZero() {
    frontLeftModule.resetAngle();
    frontRightModule.resetAngle();
    backLeftModule.resetAngle();
    backRightModule.resetAngle();

  }

  public double getYSpeed() {
    if (this.speeds != null) {
      return this.speeds.vyMetersPerSecond;
    } else {
      return 0;
    }
  }

  public double getAngularSpeed() {
    if (this.speeds != null) {
      return this.speeds.omegaRadiansPerSecond;

    }
    return 0;
  }

  public double getHeading() {
    return Gyro.gyro().getYaw();
  }

  public double getDistance() {
    return (frontLeftModule.getDistance() + frontRightModule.getDistance() + backLeftModule.getDistance() + backRightModule.getDistance()) / 4;
  }

  public Rotation2d getRotation2d() {
    // new Rotation2d();
    if (Gyro.gyro() == null) {
      return new Rotation2d().fromDegrees(0);
    } else {
      // return Rotation2d.fromDegrees(Gyro.gyro().getYaw());
      return new Rotation2d().fromDegrees(Gyro.getHeading());
    }
    // return Rotation2d.fromDegrees(Gyro.gyro().getCompassHeading());
  }

  @Override
  public void periodic() {
    odometry.update(
        getRotation2d(),
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState());

    SmartDashboard.putNumber("X Speed", getXSpeed());
    SmartDashboard.putNumber("Y Speed", getYSpeed());
    SmartDashboard.putNumber("Angular Speed", getAngularSpeed());
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    SmartDashboard.putBoolean("is Coast Mode", coast);
    // SmartDashboard.putNumber("Compass Angle", Gyro.gyro().getRotation2d().getDegrees());

    SmartDashboard.updateValues();
    // This method will be called once per scheduler run
  }
}

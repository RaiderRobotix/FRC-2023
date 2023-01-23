// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveBase;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.commands.Drive;
import frc.robot.subsystems.Gyro;

public class SwerveWheelController extends SubsystemBase implements Constants {

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

  private static SwerveDriveOdometry odometry;

  

  private PIDController angleController = new PIDController(robotangleKd, robotangleKi,
      robotangleKd);

  private static SwerveModulePosition[] driveModules = new SwerveModulePosition[4];  
  
  private static SwerveWheel[] modules =  new SwerveWheel[4];

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

    this.modules[0] = this.frontLeftModule;
    this.modules[1] = this.frontRightModule;
    this.modules[2] = this.backLeftModule;
    this.modules[3] = this.backRightModule;

    this.driveModules[0] = this.frontLeftModule.getPosition();
    this.driveModules[1] = this.frontRightModule.getPosition();
    this.driveModules[2] = this.backLeftModule.getPosition();
    this.driveModules[3] = this.backRightModule.getPosition();

    this.odometry = new SwerveDriveOdometry(kDriveKinematics, getRotation2d(), driveModules, new Pose2d());


    angleController.enableContinuousInput(0, 360);
    resetMotors();
  }

  public static void reset(){
    resetMotors();
    resetOdometry(new Pose2d());
    Gyro.gyro().reset();
    Gyro.gyro().calibrate();
    resetAngle();
  }
  public static void resetMotors() {
    for(SwerveWheel module : modules){
      module.resetMotors();
    }
  }

  //Resets the CANcoder angles to 0
  public static void resetAngle() {
    for(SwerveWheel module : modules){
      module.resetAngle();
    }
  }  

  public static void toggleField() {
    fieldCentric ^= true;
  }

  public static void stopMotors() {
    for(SwerveWheel module : modules){
      module.stop();
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public static void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), driveModules, pose);
  }

  public static void toggleCoast() {
    coast ^= true;
    for(SwerveWheel module : modules){
      module.setNeutralMode(coast);
    }
  }

  public void setSpeed(double x, double y, double delta) {
    SmartDashboard.putNumber("DesiredXSpeed", x);
    SmartDashboard.putNumber("DesiredYSpeed", y);
    SmartDashboard.putBoolean("Field Centric", fieldCentric );
    if (this.fieldCentric) {
      this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());
    } else {
      this.speeds = new ChassisSpeeds(x, y, delta);
    }
    
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setState(moduleStates);
  }

  public void setSpeed(double speed, boolean sideToside, boolean fieldCentric) {
    double x = 0;
    double y = 0;
    double delta = 0;
    if(sideToside){
      x = speed;
    } else {
      y = speed;
    }
    if (fieldCentric) {
      this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());
    } else {
      this.speeds = new ChassisSpeeds(x, y, delta);
    }
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setState(moduleStates);
  }

  public void setSpeed(double x, double y, double delta, boolean fieldCentric) {
    SmartDashboard.putNumber("DesiredXSpeed", x);
    SmartDashboard.putNumber("DesiredYSpeed", y);
    if (fieldCentric) {
      this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());
    } else {
      this.speeds = new ChassisSpeeds(x, y, delta);
    }
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setState(moduleStates);
  }

  public void setState(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxAttainableSpeed);
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

  public void setHeading(double angle, double maxSpeed) {
    setSpeed(0, 0, angleController.calculate(Gyro.getHeading(), angle));
  }

  // Sets the angle of all the wheel to angle
  public static void setAngle(double angle) {
    for(SwerveWheel module : modules){
      module.setSteerAngle(angle);
    }
  }

  public double getXSpeed() {
    if (this.speeds != null) {
      return this.speeds.vxMetersPerSecond;
    } else {
      return 0;
    }
  }

  public void setSteerZero() {
    for(SwerveWheel module : modules){
      module.resetAngle();
    }
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

  public static Rotation2d getRotation2d() {
    // new Rotation2d();
    
    if (Gyro.gyro() == null) {
      return new Rotation2d().fromDegrees(0);
    } else {
      // return Rotation2d.fromDegrees(Gyro.gyro().getYaw());
      return new Rotation2d().fromDegrees(Gyro.getHeading());
    }
    // return Rotation2d.fromDegrees(Gyro.gyro().getCompassHeading());
  }

  public double getDistance(){
    return frontLeftModule.getDriveDistance();
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(),driveModules);

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  private static SwerveWheel[] modules = new SwerveWheel[4];

  private static SwerveDriveOdometry odometry;

  // private SwerveModulePositions[] locations;

  private PIDController angleController = new PIDController(robotangleKd, robotangleKi,
      robotangleKd);

  /** Creates a new drivebase. */
  public SwerveWheelController() {

    // Location of modules relative to the centre of the robot

    this.frontLeftModule = new SwerveWheel(frontLeftDriveID, frontLeftSteerID, frontLeftEncoderID, "Front Left");
    this.frontRightModule = new SwerveWheel(frontRightDriveID, frontRightSteerID, frontRightEncoderID,
        "Front Right");
    this.backLeftModule = new SwerveWheel(backLeftDriveID, backLeftSteerID, backLeftEncoderID, "Back Left");
    this.backRightModule = new SwerveWheel(backRightDriveID, backRightSteerID, backRightEncoderID, "Back Right");

    this.modules[0] = this.frontLeftModule;
    this.modules[1] = this.frontRightModule;
    this.modules[2] = this.backLeftModule;
    this.modules[3] = this.backRightModule;
    
    // Creates kinematics object using the above module location
    // TODO fill out missing variables

    this.odometry = new SwerveDriveOdometry(kDriveKinematics,
    getRotation2d(),
    getModulePositions());

    angleController.enableContinuousInput(0, 360);
    resetMotors();
    resetEncoders();
  }
  
  //Toggles between field centric and robot centric
  public static void toggleField() {
    fieldCentric ^= true;
  }

  //Toggles drive motors between coast and brake
  public static void toggleCoast() {
    coast ^= true;
    for(SwerveWheel module : modules){
      module.setNeutralMode(coast);
    }
  }

  public static void reset(){
    resetMotors();
    resetEncoders();
    resetOdometry(new Pose2d());
    Gyro.gyro().reset();
    Gyro.gyro().calibrate();
  }

  //Sets Steer angle to 0
  public static void resetMotors() {
    for(SwerveWheel module : modules){
      module.resetMotors();
    }
  }

  //Sets encoders to 0
  public static void resetEncoders(){
    for(SwerveWheel module : modules){
      module.resetEncoder();
    }
  }

   //Resets the robot's odometry to any position
   public static void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  //Sets both steer and drive motors to zero speed
  public static void stopMotors() {
    for(SwerveWheel module : modules){
      module.stop();
    }
  }

  //Gets the current robot's position
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Pose2d resetPose(){
    return new Pose2d(0,0,new Rotation2d());
  }

  //Gets the current X speed of the robot
  public double getXSpeed() {
    if (this.speeds != null) {
      return this.speeds.vxMetersPerSecond;
    } else {
      return 0;
    }
  }

  
  //Gets the current Y speed of the robot
  public double getYSpeed() {
    if (this.speeds != null) {
      return this.speeds.vyMetersPerSecond;
    } else {
      return 0;
    }
  }

  //Gets the current rotational speed of the robot
  public double getAngularSpeed() {
    if (this.speeds != null) {
      return this.speeds.omegaRadiansPerSecond;

    }
    return 0;
  }

  public double getDistance() {
    return frontLeftModule.getDistance();
  }

  //Returns the current gyro heading as a rotation2d
  //if no gyro is detected returns 0 degrees
  public static Rotation2d getRotation2d() {
    if (Gyro.gyro() == null) {
      return new Rotation2d().fromDegrees(0);
    } else {
      return new Rotation2d().fromDegrees(Gyro.getHeading());
    }
  }

  //Gets the swervemodule positions
  public static SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      backRightModule.getPosition()};
  }

  // Sets the angle of all the wheel to angle
  public static void setAngle(double angle) {
    for(SwerveWheel module : modules){
      module.setSteerAngle(angle);
    }
  }
  
  //Sets the robot's speed with a given X speed, Y Speed and rotation speed
  public void setSpeed(double x, double y, double delta) {
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
  
  //Sets the robot's speed with a given X speed, Y Speed and rotation speed
  //and a field centric override
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
  //Sets the state of an array of SwerveModuleStates, with a max speed
  public void setState(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxAttainableSpeed);
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);

    

  }

  



  

  
  

  
  @Override
  public void periodic() {

    
    odometry.update(
        getRotation2d(),
        getModulePositions());

    SmartDashboard.putNumber("X Speed", getXSpeed());
    SmartDashboard.putNumber("Y Speed", getYSpeed());
    SmartDashboard.putNumber("Angular Speed", getAngularSpeed());
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    SmartDashboard.putBoolean("is Coast Mode", coast);

    SmartDashboard.updateValues();
  }
}

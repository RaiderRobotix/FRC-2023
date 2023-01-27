// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.module.Configuration;
import java.text.DecimalFormat;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import frc.robot.Constants;
import frc.robot.commands.Drive;


public class SwerveWheel extends SubsystemBase implements Constants {

  private CANSparkMax driveMotor;
  private CANSparkMax steerMotor;
  private WPI_CANCoder encoder;

  private RelativeEncoder driveEncoder;
  private double driveDistance;

  private RelativeEncoder steerEncoder;
  private double steerDistance;

  private SparkMaxPIDController steeringPid;
  

  private String name;

  private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);
  private PIDController driveController = new PIDController(driveKp, driveKi, driveKd);

  private DecimalFormat df = new DecimalFormat("###.##");


  

  /** Creates a new ExampleSubsystem. */
  public SwerveWheel(int driveID, int steerID, int encoderID, String name) {
    this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    this.steerMotor = new CANSparkMax(steerID,  MotorType.kBrushless);
    this.encoder = new WPI_CANCoder(encoderID);
    this.name = name;

    this.driveMotor.restoreFactoryDefaults();
    this.steerMotor.restoreFactoryDefaults();
    this.encoder.configFactoryDefault();

    this.angleController.enableContinuousInput(0, 360);

    this.driveMotor.setSmartCurrentLimit(80);
    
    // this.steeringPid = steerMotor.getPIDController();
    // this.steeringPid.setOutputRange(0, 360);

    // this.steeringPid.setP(angleKp);
    // this.steeringPid.setI(angleKi);
    // this.steeringPid.setD(angleKd);

    this.driveEncoder = driveMotor.getEncoder();
    this.steerEncoder = steerMotor.getEncoder();

    

  
    this.driveDistance = this.getDriveEncoder();
    driveMotor.setIdleMode(IdleMode.kBrake);
    if (encoder.getDeviceID() == 0) {
      encoder.configMagnetOffset(frontLeftEncoderOffset);
    }
    if (encoder.getDeviceID() == 1) {
      encoder.configMagnetOffset(frontRightEncoderOffset);
    }
    if (encoder.getDeviceID() == 2) {
      encoder.configMagnetOffset(backleftEncoderOffset);
    }
    if (encoder.getDeviceID() == 3) {
      encoder.configMagnetOffset(backRightEncoderOffset);
    }
    configureSteerMotor();
  }

  public void configureSteerMotor(){
    steerMotor.restoreFactoryDefaults();
    steerMotor.clearFaults();
    steerMotor.setIdleMode(IdleMode.kBrake);
    steerMotor.setSmartCurrentLimit(20);
    steerEncoder.setPositionConversionFactor(360 * (14/50) * (10/60));
    driveEncoder.setVelocityConversionFactor(500);
    steerMotor.enableVoltageCompensation(12);
    steerMotor.burnFlash();

    steerEncoder.setPosition(encoder.getAbsolutePosition());
  }

  public void resetMotors(){

  }

  public void setNeutralMode(boolean coast){
    if(coast){
      driveMotor.setIdleMode(IdleMode.kCoast);
      steerMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setIdleMode(IdleMode.kBrake);
      steerMotor.setIdleMode(IdleMode.kBrake);
    }
  }


  public WPI_CANCoder getEncoder() {
    return this.encoder;
  }

  private double getDriveEncoder() { 
    return (this.driveEncoder.getPosition() * kUnitsPerRevoltion); 
  }

  public double getDriveDistance() { 
    return this.getDriveEncoder() - this.driveDistance; 
  }

  private double getSteerEncoder() { 
    return (this.steerEncoder.getPosition() * kUnitsPerRevoltion); 
  }

  public double getSteerDistance() { 
    return this.getDriveEncoder() - this.steerDistance; 
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(this.getDriveDistance(),new Rotation2d(getSteerAngle().getDegrees()));
  }

  public String getName(){
    return this.name;
  }

  public void stop(){
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }

  public double getDriveSpeed(){
      return driveEncoder.getVelocity();
  }

  public void setDriveSpeed(double speed){
    // System.out.println(speed);
    driveMotor.set(speed);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getSteerAngle().getDegrees()));
  }

  public Rotation2d getSteerAngle(){
    // double encoderValue = encoder.getAbsolutePosition();
    double encoderValue = steerEncoder.getPosition();
    return new Rotation2d().fromDegrees(encoderValue);
  }

  public void setSteerAngle(double angle) {
    System.out.println(angle);
    steerMotor.setVoltage(angleController.calculate(getSteerAngle().getDegrees(), angle) * RobotController.getBatteryVoltage());
  }

  public void resetAngle(){
    steerEncoder.setPosition(encoder.getAbsolutePosition());
  }

  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerAngle());
    setDriveSpeed(state.speedMetersPerSecond);
    setSteerAngle(state.angle.getDegrees());
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber(this.getName() + " Steer Angle", 
    Double.parseDouble(df.format(steerMotor.get())));
    
    SmartDashboard.putNumber(this.getName() + " Drive Speed", 
    Double.parseDouble(df.format(driveMotor.get())));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

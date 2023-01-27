// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.module.Configuration;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

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
  private WPI_CANCoder CANencoder;

  private RelativeEncoder driveEncoder;
  private double driveDistance;

  private RelativeEncoder steerEncoder;
  private double steerDistance;

  private SparkMaxPIDController steeringPid;
  

  private String name;

  private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);
  private PIDController driveController = new PIDController(driveKp, driveKi, driveKd);

  private DecimalFormat df = new DecimalFormat("###.##", new DecimalFormatSymbols(Locale.US));


  

  /** Creates a new ExampleSubsystem. */
  public SwerveWheel(int driveID, int steerID, int encoderID, String name) {
    this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    this.steerMotor = new CANSparkMax(steerID,  MotorType.kBrushless);
    this.CANencoder = new WPI_CANCoder(encoderID);
    this.driveEncoder = driveMotor.getEncoder();
    this.steerEncoder = steerMotor.getEncoder();
    this.name = name;

    
    this.angleController.enableContinuousInput(0, 360);
    
    
    // this.steeringPid = steerMotor.getPIDController();
    // this.steeringPid.setOutputRange(0, 360);
    
    // this.steeringPid.setP(angleKp);
    // this.steeringPid.setI(angleKi);
    // this.steeringPid.setD(angleKd);
    
    
    if (CANencoder.getDeviceID() == 0) {
      CANencoder.configMagnetOffset(frontLeftEncoderOffset);
    }
    if (CANencoder.getDeviceID() == 1) {
      CANencoder.configMagnetOffset(frontRightEncoderOffset);
    }
    if (CANencoder.getDeviceID() == 2) {
      CANencoder.configMagnetOffset(backleftEncoderOffset);
    }
    if (CANencoder.getDeviceID() == 3) {
      CANencoder.configMagnetOffset(backRightEncoderOffset);
    }
    configureSteerMotor();
  }
  
  public void configureSteerMotor(){
    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();
    CANencoder.configFactoryDefault();
    
    steerMotor.restoreFactoryDefaults();
    steerMotor.clearFaults();
    steerMotor.setIdleMode(IdleMode.kBrake);
    steerMotor.setSmartCurrentLimit(20);
    steerEncoder.setPositionConversionFactor(360 * (14/50) * (10/60));
    steerEncoder.setVelocityConversionFactor(360 * (14/50) * (10/60) / 60);
    steerMotor.enableVoltageCompensation(12);
    steerMotor.burnFlash();
    
    
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(80);
    driveMotor.enableVoltageCompensation(12);
    driveEncoder.setVelocityConversionFactor(kWheelDiameter * Math.PI / kGearRatio);
    driveEncoder.setPositionConversionFactor(kWheelDiameter * Math.PI / kGearRatio / 60);
    driveMotor.burnFlash();
    
    steerEncoder.setPosition(CANencoder.getAbsolutePosition());
  }
  
  public void resetSteerMotor(){
    setSteerAngle(0);
  }

  public void resetEncoders(){
    steerEncoder.setPosition(0);
    driveEncoder.setPosition(0);
  }

  public void absoluteAngle(){
    steerEncoder.setPosition(CANencoder.getAbsolutePosition());
  }

  public void stop(){
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }

  
  public String getName(){
    return this.name;
  }

  public double getDriveEncoderPosition() { 
    return driveEncoder.getPosition(); 
  }

  private double getDriveEncoderVelocity(){
    return driveEncoder.getVelocity();
  }

  private Rotation2d getSteerEncoderAngle(){
    return new Rotation2d().fromDegrees(steerEncoder.getPosition());
  }

  public Rotation2d getCANCoderAngle() {
    return new Rotation2d().fromDegrees(CANencoder.getAbsolutePosition());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveEncoderPosition(), new Rotation2d(getSteerEncoderAngle().getDegrees()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveEncoderVelocity(), new Rotation2d(getSteerEncoderAngle().getDegrees()));
  }

  public void setDriveSpeed(double speed){
    driveMotor.setVoltage(driveController.calculate(getDriveEncoderVelocity(), speed));
  }

  public void setSteerAngle(double angle) {
    // System.out.println(angle);
    steerMotor.setVoltage(angleController.calculate(getSteerEncoderAngle().getDegrees(), angle) * RobotController.getBatteryVoltage());
  }

  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerEncoderAngle());
    setDriveSpeed(state.speedMetersPerSecond);
    setSteerAngle(state.angle.getDegrees());
  }

  public void setNeutralMode(boolean coast){
    if(coast){
      driveMotor.setIdleMode(IdleMode.kCoast);
      // steerMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setIdleMode(IdleMode.kBrake);
      // steerMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + " Steer Angle", 
    Double.parseDouble(df.format(steerEncoder.getPosition())));
    
    SmartDashboard.putNumber(getName() + " Drive Speed", 
    Double.parseDouble(df.format(driveEncoder.getVelocity())));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

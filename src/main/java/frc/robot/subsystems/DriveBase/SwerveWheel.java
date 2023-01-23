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

    this.driveMotor.setSmartCurrentLimit(55);
    this.steerMotor.setSmartCurrentLimit(55);
    
    this.steeringPid = steerMotor.getPIDController();
    this.steeringPid.setOutputRange(0, 360);

    this.steeringPid.setP(angleKp);
    this.steeringPid.setI(angleKi);
    this.steeringPid.setD(angleKd);

    this.driveEncoder = driveMotor.getEncoder();
    
    driveMotor.clearFaults();
    steerMotor.clearFaults();
  
    this.driveDistance = this.getDriveEncoder();
    driveMotor.setIdleMode(IdleMode.kBrake);
    steerMotor.setIdleMode(IdleMode.kBrake);

    if (encoder.getDeviceID() == 1) {
      encoder.configMagnetOffset(frontRightEncoderOffset);
    }
    if (encoder.getDeviceID() == 2) {
      encoder.configMagnetOffset(backleftEncoderOffset);
    }
    if (encoder.getDeviceID() == 3) {
      encoder.configMagnetOffset(backRightEncoderOffset);
    }
    if (encoder.getDeviceID() == 0) {
      encoder.configMagnetOffset(frontLeftEncoderOffset);
    }
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
      return driveMotor.get();
  }

  public void setDriveSpeed(double speed){
    driveMotor.set(speed);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getSteerAngle().getDegrees()));
  }

  public Rotation2d getSteerAngle(){
    double encoderValue = encoder.getAbsolutePosition();
    return new Rotation2d().fromDegrees(encoderValue);
  }

  public void setSteerAngle(double angle) {
   
    // steerMotor.set(angleController.calculate(getSteerAngle().getDegrees(), angle));
    steeringPid.setReference(angle, ControlType.kPosition);
  }

  public void resetAngle(){
    encoder.setPositionToAbsolute(0);
  }

  public void setDesiredState(SwerveModuleState state) {
    
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getSteerAngle());
    setDriveSpeed(state.speedMetersPerSecond);
    setSteerAngle(state.angle.getDegrees());

    // System.out.println(this.name + " " + getSteerAngle().getDegrees());

  }



  /*public void setSteerAngle(double angle) {
    steerMotor.set(angleController.calculate(getSteerAngle().getDegrees(), angle));
    // steerMotor.set(angle);
  }*/

  @Override
  public void periodic() {
    SmartDashboard.putNumber(this.getName() + " Steer Angle",
    Double.parseDouble(df.format(this.getSteerAngle().getDegrees())));

    SmartDashboard.putNumber(this.getName() + " Drive Speed",
    Double.parseDouble(df.format(this.getDriveSpeed())));

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

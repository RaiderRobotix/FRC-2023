// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.module.Configuration;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.sensors.WPI_CANCoder;


public class SwerveWheel extends SubsystemBase {

  private CANSparkMax driveMotor;
  private CANSparkMax steerMotor;
  private WPI_CANCoder encoder;

  private String name;


  //private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);
  

  /** Creates a new ExampleSubsystem. */
  public SwerveWheel(int driveID, int steerID, int encoderID, String name) {
    this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    this.steerMotor = new CANSparkMax(steerID,  MotorType.kBrushless);
    this.encoder = new WPI_CANCoder(encoderID);
    this.name = name;

    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();
    encoder.configFactoryDefault();

    driveMotor.setSmartCurrentLimit(20);
    steerMotor.setSmartCurrentLimit(20);
      
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
    setDriveSpeed(speed);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getSteerAngle().getDegrees()));
  }

  public Rotation2d getSteerAngle(){
    double encoderValue = encoder.getAbsolutePosition();
    return new Rotation2d().fromDegrees(encoderValue);
  }



  /*public void setSteerAngle(double angle) {
    steerMotor.set(angleController.calculate(getSteerAngle().getDegrees(), angle));
    // steerMotor.set(angle);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

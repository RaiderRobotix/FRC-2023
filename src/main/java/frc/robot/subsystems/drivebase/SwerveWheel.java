// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveWheel extends SubsystemBase implements Constants {
  /** Creates a new SwerveWheel. */


  private WPI_TalonFX driveMotor;
  private WPI_TalonFX steerMotor;
  private WPI_CANCoder encoder;

  private String name;

  private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);
  private PIDController driveController = new PIDController(driveKp, driveKi, driveKd);


  //Initalizes the format that will limit decimal places in doubles
  //To use Double.parseDouble(df.format(DOUBLE))
  private DecimalFormat df = new DecimalFormat("###.##", new DecimalFormatSymbols(Locale.US));

  public SwerveWheel(
      int driveID,
      int steerID,
      int encoderID, String name) {
    this.driveMotor = new WPI_TalonFX(driveID);
    this.steerMotor = new WPI_TalonFX(steerID);
    this.encoder = new WPI_CANCoder(encoderID);
    this.name = name;

    //Sets motors and CANcoders to factory settings before changing them
    this.driveMotor.configFactoryDefault();
    this.steerMotor.configFactoryDefault();
    this.encoder.configFactoryDefault();

    //Limits the current to the drive and steer motors
    this.driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    this.steerMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    this.steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));

    this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    this.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    //Sets both motors to be in brake mode by default
    steerMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    
    //Allows the angle PID contrller to roll over from 0 to 360 
    angleController.enableContinuousInput(0, 360);

    //Initalizes the indiviual CANcoder's specfic offset
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

  public void resetMotors() {
    setSteerAngle(0);
  }

  public void resetEncoder(){
    driveMotor.setSelectedSensorPosition(0);
  }

  public void resetAngle(){
    encoder.setPositionToAbsolute(0);
  }

  public void stop() {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  public WPI_CANCoder getEncoder() {
    return this.encoder;
  }

  public String getName() {
    return this.name;
  }

  public double getDriveSpeed() {
    return driveMotor.getSelectedSensorVelocity(0) / kUnitsPerRevoltion;
    // return (kMaxRPM / 600) * (driveMotor.getSelectedSensorVelocity() /
    // kGearRatio);
  }

  //units in metres
  public double getDistance(){
    return driveMotor.getSelectedSensorPosition(0) / (kUnitsPerRevoltion * kGearRatio) * (Math.PI * kWheelDiameter);
  }

  public Rotation2d getSteerAngle() {
    double encoderValue = encoder.getAbsolutePosition();
    return new Rotation2d().fromDegrees(encoderValue);
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDistance(), getSteerAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getSteerAngle().getDegrees()));
  }

  public void setNeutralMode(boolean coast) {    
    if (coast) {
      driveMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public void setDriveSpeed(double speed) {
    // this.driveMotor.set(driveController.calculate(getDriveSpeed(), speed));
    // System.out.println(speed);
    SmartDashboard.putNumber("Desired Wheel Speed", speed);
    this.driveMotor.set(speed);
  }
  
  public void setSteerAngle(double angle) {
    SmartDashboard.putNumber(this.getName() + "Desired Angle", angle);
    steerMotor.set(angleController.calculate(getSteerAngle().getDegrees(), angle));
  }

  public void setDesiredAngle(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerAngle());
    setSteerAngle(state.angle.getDegrees());
  }


  public void setDesiredState(SwerveModuleState state) {
    // state = SwerveModuleState.optimize(state, getSteerAngle());
    setDriveSpeed(state.speedMetersPerSecond);
    setSteerAngle(state.angle.getDegrees());
  }

  @Override
  public void periodic() {
    //Double.parseDouble(df.format()) limits the number of decimal places
    //because I (Zach) thinks too many is ugly and unhelpful
    SmartDashboard.putNumber(this.getName() + " Steer Angle",
        Double.parseDouble(df.format(this.getSteerAngle().getDegrees())));

    SmartDashboard.putNumber(this.getName() + " Drive Speed",
        Double.parseDouble(df.format(this.getDriveSpeed())));

    // This method will be called once per scheduler run
  }
}

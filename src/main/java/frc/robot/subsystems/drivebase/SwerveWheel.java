// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.drive;

public class SwerveWheel extends SubsystemBase implements drivebaseConstants, Constants {
  /** Creates a new SwerveWheel. */

  private WPI_TalonFX driveMotor;
  private WPI_TalonFX steerMotor;
  private WPI_CANCoder encoder;

  private String name;

  private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);
  private PIDController driveController = new PIDController(driveKp, driveKi, driveKd);

  private DecimalFormat df = new DecimalFormat("###.##");

  public SwerveWheel(
      int driveID,
      int steerID,
      int encoderID, String name) {
    this.driveMotor = new WPI_TalonFX(driveID);
    this.steerMotor = new WPI_TalonFX(steerID);
    this.encoder = new WPI_CANCoder(encoderID);
    this.name = name;

    this.driveMotor.configFactoryDefault();
    this.steerMotor.configFactoryDefault();
    this.encoder.configFactoryDefault();

    this.driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    this.steerMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    this.steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    // this.encoder.configAbsoluteSensorRange(range);

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    // this.driveMotor.configAllSettings(configuration);
    // this.steerMotor.configAllSettings(configuration);

    angleController.enableContinuousInput(0, 360);

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
    // System.out.println(getSteerAngle());
    setSteerAngle(0);
  }

  public void setNeutralMode(boolean coast) {
    if (coast) {
      driveMotor.setNeutralMode(NeutralMode.Coast);
      steerMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralMode.Brake);
      steerMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public WPI_CANCoder getEncoder() {
    return this.encoder;
  }

  public String getName() {
    return this.name;
  }

  public void setDesiredState(SwerveModuleState state) {
    // System.out.println(state);
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getSteerAngle());
    setDriveSpeed(state.speedMetersPerSecond);
    setSteerAngle(state.angle.getDegrees());

    // System.out.println(this.name + " " + getSteerAngle().getDegrees());

  }

  public void stop() {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  public void resetAngle(){
    encoder.setPositionToAbsolute(0);
  }

  public void setDesiredAngle(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerAngle());
    setSteerAngle(state.angle.getDegrees());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getSteerAngle().getDegrees()));
  }

  public double getDriveSpeed() {
    return driveMotor.getSelectedSensorVelocity(0) / kUnitsPerRevoltion;
    // return (kMaxRPM / 600) * (driveMotor.getSelectedSensorVelocity() /
    // kGearRatio);
  }

  public Rotation2d getSteerAngle() {
    double encoderValue = encoder.getAbsolutePosition();
    // double encoderValue = steerMotor.getSelectedSensorPosition(0);
    // System.out.println(this.name + " " + encoderValue);
    // encoderValue = (360 * encoderValue / kUnitsPerRevoltion) % 360;
    // return new Rotation2d(encoderValue);
    return new Rotation2d().fromDegrees(encoderValue);
  }

  public void setDriveSpeed(double speed) {
    // this.driveMotor.set(driveController.calculate(getDriveSpeed(), speed));
    this.driveMotor.set(speed);
  }

  public void setSteerAngle(double angle) {
    SmartDashboard.putNumber(this.getName() + "Desired Angle", angle);
    steerMotor.set(angleController.calculate(getSteerAngle().getDegrees(), angle));
    // steerMotor.set(angle);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber(this.getName() + " Steer Angle",
        Double.parseDouble(df.format(this.getSteerAngle().getDegrees())));

    SmartDashboard.putNumber(this.getName() + " Drive Speed",
        Double.parseDouble(df.format(this.getDriveSpeed())));

    // This method will be called once per scheduler run
  }
}

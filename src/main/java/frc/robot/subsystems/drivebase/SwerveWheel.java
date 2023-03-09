// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

import org.opencv.core.Mat;

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
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.commands.drive;

public class SwerveWheel extends SubsystemBase implements Constants {
  /** Creates a new SwerveWheel. */

  private WPI_TalonFX driveMotor;
  private WPI_TalonFX steerMotor;
  private WPI_CANCoder encoder;

  private String name;

  private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);
  private PIDController driveController = new PIDController(driveKp, driveKi, driveKd);

  private DecimalFormat df = new DecimalFormat("###.##", new DecimalFormatSymbols(Locale.US));
  
  private ShuffleboardTab drivebase = Shuffleboard.getTab("driveBase");

  private GenericEntry steerAngleTableEntry;
  private GenericEntry driveSpeedTableEntry;
  private GenericEntry desiredAngleTableEntry;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4_L3.getDriveReduction() *
      SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;
  
  
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
      
      steerAngleTableEntry = drivebase.add(this.name + "Steer Angle", 0).getEntry();
      driveSpeedTableEntry = drivebase.add(this.name + "Drive Speed", 0).getEntry();
      desiredAngleTableEntry = drivebase.add(this.name + "Desired Angle", 0).getEntry();
      
      angleController.enableContinuousInput(-Math.PI, Math.PI);

    if (encoder.getDeviceID() == 0) {
      encoder.configMagnetOffset(frontLeftEncoderOffset);
    }
    if (encoder.getDeviceID() == 1) {
      encoder.configMagnetOffset(frontRightEncoderOffset);
    }
    if (encoder.getDeviceID() == 2) {
      encoder.configMagnetOffset(backLeftEncoderOffset);
    }
    if (encoder.getDeviceID() == 3) {
      encoder.configMagnetOffset(backRightEncoderOffset);
    }
  }

  public void resetMotors() {
    // System.out.println(getSteerAngle());
    setSteerAngle(0);
  }

  public void setNeutralMode(boolean coast) {
    if (coast) {
      driveMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public WPI_CANCoder getEncoder() {
    return this.encoder;
  }

  public String getName() {
    return this.name;
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), getSteerAngle());
  }

  public void set(double percentage, double radians){
    setSteerAngle(radians);
    setDriveSpeed(percentage);
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getSteerAngle());
    setDriveSpeed(state.speedMetersPerSecond);
    setSteerAngle(state.angle.getDegrees());
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
    set(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND, state.angle.getRadians());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getSteerAngle().getDegrees()));
  }

  public double getDriveSpeed() {
    return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), kWheelDiameter, kGearRatio);
  }

  public double getDrivePosition(){
    return Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), kWheelDiameter, kGearRatio);
  }

  public Rotation2d getSteerAngle() {
    double encoderValue = encoder.getAbsolutePosition();
    return new Rotation2d().fromDegrees(encoderValue);
  }

  public void setDriveSpeed(double speed) {
    // this.driveMotor.set(driveController.calculate(getDriveSpeed(), speed));
    this.driveMotor.set(speed);
  }

  public void setSteerAngle(double radians) {
    // desiredAngleTableEntry.setDouble(Double.parseDouble(df.format(radians)));
    // steerMotor.set(angleController.calculate(getSteerAngle().getDegrees(), radians));
    // desiredAngleTableEntry.setDouble(Double.parseDouble(df.format(radians)));
    steerMotor.set(angleController.calculate(getSteerAngle().getRadians(), radians));
  }

  @Override
  public void periodic() {
    steerAngleTableEntry.setDouble(Double.parseDouble(df.format(getSteerAngle().getDegrees())));
    driveSpeedTableEntry.setDouble(Double.parseDouble(df.format(getDriveSpeed())));
  }
}
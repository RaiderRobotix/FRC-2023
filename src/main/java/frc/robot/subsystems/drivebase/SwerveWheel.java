// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveWheel extends SubsystemBase implements drivebaseConstants, Constants {
  /** Creates a new SwerveWheel. */

  private WPI_TalonFX driveMotor;
  private WPI_TalonFX steerMotor;
  private CANCoder encoder;

  private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);

  public SwerveWheel(
      int driveID,
      int steerID,
      int encoderID, String name) {
    this.driveMotor = new WPI_TalonFX(driveID);
    this.steerMotor = new WPI_TalonFX(steerID);
    this.encoder = new CANCoder(encoderID);

    this.driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    this.steerMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    this.steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    // this.encoder.configAbsoluteSensorRange(range);

    angleController.enableContinuousInput(0, 360);

  }

  public void resetMotors(){
    System.out.println(getSteerAngle());
    setSteerAngle(0);
  }

  public void setDesiredState(SwerveModuleState state) {
    // System.out.println(state);
    state = SwerveModuleState.optimize(state, getSteerAngle());
    setDriveSpeed(state.speedMetersPerSecond / kPhysicalDriveMaxSpeed);
    setSteerAngle(state.angle.getDegrees());

  }

  public void setDesiredAngle(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerAngle());
    setSteerAngle(state.angle.getDegrees());
  }

  public double getDriveSpeed() {
    return (kMaxRPM / 600) * (driveMotor.getSelectedSensorVelocity() / kGearRatio);
  }

  public Rotation2d getSteerAngle() {
    double encoderValue = encoder.getAbsolutePosition();
    // System.out.println(encoderValue);
    encoderValue = 180 * encoderValue / kUnitsPerRevoltion;
    return new Rotation2d(encoderValue);
  }

  public void setDriveSpeed(double speed) {
    this.driveMotor.set(speed);
  }

  public void setSteerAngle(double angle) {
    System.out.println("Angle:" + encoder.getAbsolutePosition());
    steerMotor.set(angle);
    // steerMotor.set(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.commands.drive;

public class SwerveWheel extends SubsystemBase implements drivebaseConstants {
  /** Creates a new SwerveWheel. */

  private WPI_TalonFX driveMotor;
  private WPI_TalonFX steerMotor;

  private TalonFXSimCollection driveMotorSim;
  private TalonFXSimCollection steerMotorSim;

  private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);
  private PIDController driveController = new PIDController(driveKp, driveKi, driveKd);

  public SwerveWheel(
      int driveID,
      int steerID,
      int encoderID, String name) {
    this.driveMotor = new WPI_TalonFX(driveID);
    this.steerMotor = new WPI_TalonFX(steerID);
    this.driveMotorSim = this.driveMotor.getSimCollection();
    this.steerMotorSim = this.driveMotor.getSimCollection();
    angleController.enableContinuousInput(0, 360);

  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getSteerAngle()));
  }

  // TODO Convert to metres per second
  public double getDriveSpeed() {
    return this.driveMotor.getSelectedSensorVelocity();
  }

  // TODO Conver to angle or radians
  public double getSteerAngle() {
    double encoderValue = this.steerMotor.getSelectedSensorPosition();
    return (180 * encoderValue / 4096);
  }

  public void set(SwerveModuleState state) {
    SwerveModuleState desiredState = SwerveModuleState.optimize(state, new Rotation2d(getSteerAngle()));

    final double driveOutput = driveController.calculate(getDriveSpeed(), desiredState.speedMetersPerSecond);

    final double angleOutput = angleController.calculate(getSteerAngle(), desiredState.angle.getDegrees());

    // TODO fix this
    this.driveMotor.set(TalonFXControlMode.PercentOutput, driveOutput);
    this.steerMotor.set(TalonFXControlMode.PercentOutput, angleOutput);
  }

  public void setDriveSpeed(double speed) {
    this.driveMotor.set(speed);
  }

  public void setSteerAngle(double angle) {
    this.steerMotor.set(angleController.calculate(getSteerAngle(), angle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

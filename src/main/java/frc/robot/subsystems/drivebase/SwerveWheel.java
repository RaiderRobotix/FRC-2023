// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class SwerveWheel extends SubsystemBase implements drivebaseConstants {
  /** Creates a new SwerveWheel. */

  private WPI_TalonFX driveMotor;
  private WPI_TalonFX steerMotor;

  private PIDController angleController = new PIDController(angleKp, angleKi, angleKd);

  public SwerveWheel(
      int driveID,
      int steerID,
      int encoderID, String name) {
    this.driveMotor = new WPI_TalonFX(driveID);
    this.steerMotor = new WPI_TalonFX(encoderID);

    angleController.enableContinuousInput(0, 360);

  }

  public double getDriveSpeed() {
    return driveMotor.getSelectedSensorVelocity();
  }

  public double getSteerAngle() {
    double encoderValue = steerMotor.getSelectedSensorPosition();
    return (180 * encoderValue / 4096);
  }

  public void setDriveSpeed(double speed) {
    this.driveMotor.set(speed);
  }

  public void setSteerAngle(double angle) {
    steerMotor.set(angleController.calculate(getSteerAngle(), angle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

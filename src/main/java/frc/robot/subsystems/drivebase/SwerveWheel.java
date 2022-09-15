// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveWheel extends SubsystemBase implements Constants {
  /** Creates a new SwerveWheel. */

  private TalonSRX driveMotor;
  private TalonSRX steerMotor;

  public SwerveWheel(int driveID, int steerID, int encoderID, String name) {
    this.driveMotor = new TalonSRX(driveID);
    this.steerMotor = new TalonSRX(encoderID);

  }

  public double getDriveSpeed() {
    return driveMotor.getSelectedSensorVelocity();
  }

  public double getSteerAngle() {
    double encoderValue = steerMotor.getSelectedSensorPosition();

    return steerMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

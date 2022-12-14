// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.Normalizer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */

  private static AHRS ahrs;

  public Gyro() {
    Gyro.ahrs = new AHRS(Port.kUSB1);
  }

  public static AHRS gyro() {
    return ahrs;
  }

  public static double getHeading() {
    // return Math.abs(gyro().getYaw() + 180);
    return (gyro().getYaw() + 180);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

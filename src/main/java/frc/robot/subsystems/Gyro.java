// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.Normalizer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */

  public static AHRS ahrs = new AHRS(Port.kMXP);

  public Gyro() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyro().reset();
      } catch (Exception e) {

      }
    }).start();
  }

  public static AHRS gyro() {
    try {
      return ahrs;
    } catch (Exception e) {
      System.out.println("Gyro null");
      ahrs = new AHRS(Port.kMXP);
      return ahrs;
    }
  }  

  public static double getHeading() {
    // return gyro().getYaw();
    return gyro().getFusedHeading();
    // return 0.0;
    // return Math.abs();
    // return Math.IEEEremainder(Math.abs(gyro().getAngle()), 360);
  }

  public static void resetGyro(){
    gyro().reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", getHeading());
    // This method will be called once per scheduler run
  }
}
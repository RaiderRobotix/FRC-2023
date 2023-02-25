// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase implements Constants{
  /** Creates a new Arm. */

  private static AnalogPotentiometer tenTurnPot;
  private static WPI_TalonFX motor;
  private static PIDController pid;

  public Arm() {
    tenTurnPot = new AnalogPotentiometer(kArmPotentiometer);
    motor = new WPI_TalonFX(kArmTalonFX);
    pid = new PIDController(armKp, armKi, armKd);
  }

  public static void setMotorPID(double distance){
    double pidValue = pid.calculate(getSensor(), distance);
    motor.set(pidValue);
  }

  public static void setMotor(double speed){
   // if(getSensor() >= kArmMaxLength || getSensor() >= kArmMinLength){
   //   motor.set(0);
   // }
    motor.set(speed);
  } 

  public static void setUpperRow(){
    setMotorPID(kUpperRowLength);
  }

  public static void setMidRow(){
    setMotorPID(kMidRowLength);
  }

  public static void setLowerRow(){
    setMotorPID(kFloorLength);
  }

  public static void setFloor(){
    setMotorPID(kFloorLength);
  }


  public static double getSensor(){
    return tenTurnPot.get();
    // return motor.getSelectedSensorPosition() / (kArmDistancePerRotation * kArmGearRatio);
  }

  public static boolean getSensorMax() {
    return (getSensor() >= kArmMaxLength);
  }

  public static boolean getSensorLow() {
    return (getSensor() <= kArmMinLength);
  }

  public static boolean isUpperRow(){
    return Math.abs(kUpperRowLength - getSensor()) < 0.5;
  }

  public static boolean isMidRow() {
    return Math.abs(kMidRowLength - getSensor()) < 0.05;
  }

  public static boolean isLowRow() {
    return Math.abs(kFloorLength - getSensor()) < 0.05;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("isUpper", isUpperRow());
    SmartDashboard.putNumber("Arm Encoder", getSensor());

    // This method will be called once per scheduler run
  }
}

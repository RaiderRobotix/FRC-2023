// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase implements Constants {
  private static AnalogPotentiometer tenTurnPot;
  private static WPI_TalonFX motor;
  private static PIDController pid;
  /** Creates a new Elevator. */
  public Elevator() {
    this.tenTurnPot = new AnalogPotentiometer(kElevatorPotentiometer);
    this.motor = new WPI_TalonFX(kElevatorTalonFX);
    this.pid = new PIDController(elevatorKp, elevatorKi, elevatorKd);
    // pid.enableContinuousInput(0, 1);
  }

  public static void setMotorPID(double height){
    if(getSensor() >= kElevatorMaxHeight || getSensor() <= kElevatorMinHeight){
      motor.set(0);
    }
    motor.set(pid.calculate(getSensor(), height));
  }

  public static void setMotor(double speed){
    // if(getSensor() <= kElevatorMaxHeight || getSensor() >= kElevatorMinHeight){
    //   motor.set(0);
    // } else {
    //   motor.set(speed);
    // }
    motor.set(speed);
  }

  public static void setUpperRow(){
    setMotorPID(kUpperRowHeight);
  }

  public static void setMidRow(){
    setMotorPID(kMidRowHeight);
  }

  public static void setFloor(){
    setMotorPID(kFloorHeight);
  }

  public static double getSensor(){
    return tenTurnPot.get();
    // return -motor.getSelectedSensorPosition() / kEleveatorGearRatio;
  }

  public static boolean getSensorMax(){
    return (getSensor() >= kElevatorMaxHeight);
  }

  public static boolean getSensorLow() {
    return (getSensor() <= kElevatorMinHeight);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder", getSensor());
    SmartDashboard.putBoolean("getLow", getSensorLow());
    SmartDashboard.putBoolean("getHigh", getSensorMax());
    // This method will be called once per scheduler run
  }
}
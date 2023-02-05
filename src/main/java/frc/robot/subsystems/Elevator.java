// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase implements Constants {
  private static DutyCycleEncoder boreThrough;
  private static WPI_TalonFX motor;
  private static PIDController pid;
  /** Creates a new Elevator. */
  public Elevator() {
    this.boreThrough = new DutyCycleEncoder(kElevatorEncoder);
    this.boreThrough.setDistancePerRotation(kElevatorDistancePerRotation);
    this.motor = new WPI_TalonFX(kElevatorTalonFX);
    this.pid = new PIDController(elevatorKp, elevatorKi, elevatorKd);
    // pid.enableContinuousInput(0, 1);
  }

  public static void setMotorPID(double height){
    motor.set(pid.calculate(boreThrough.getAbsolutePosition(), height));
  }

  public static void setMotor(double speed){
    motor.set(speed);
  }

  public static void setElevatorUpperRow(){
    setMotorPID(kUpperRowHeight);
  }
  public static void setElevatorMidRow(){
    setMotorPID(kMidRowHeight);
  }
  public static void setElevatorLowerRow(){
    setMotorPID(kLowerRowHeight);
  }

  public static double getSensor(){
    return boreThrough.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Bore Through", boreThrough.getAbsolutePosition());
    // This method will be called once per scheduler run
  }
}

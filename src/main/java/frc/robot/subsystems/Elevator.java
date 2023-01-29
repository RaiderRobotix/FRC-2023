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

public class Elevator extends SubsystemBase {
  private static DutyCycleEncoder boreThrough;
  private static WPI_TalonFX motor;
  private static PIDController pid;
  /** Creates a new Elevator. */
  public Elevator() {
    this.boreThrough = new DutyCycleEncoder(1);
    this.motor = new WPI_TalonFX(0);
    this.pid = new PIDController(0.1, 0, 0);
    pid.enableContinuousInput(0, 1);
  }

  public static void setMotor(){
    System.out.println("passed");
    motor.set(pid.calculate(boreThrough.getAbsolutePosition(), 1));
  }

  public static void setMotor(double speed){
    motor.set(speed);
  }

  public static double getSensor(){
    return boreThrough.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Bore THrough", boreThrough.getAbsolutePosition());
    // This method will be called once per scheduler run
  }
}

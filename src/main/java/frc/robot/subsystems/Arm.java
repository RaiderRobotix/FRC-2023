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
    if(getSensor() >= kArmMaxHeight || getSensor() <= kArmMinHeight){
      motor.set(0);
    }
    motor.set(pid.calculate(getSensor(), distance));
  }

  public static void setMotor(double speed){
    if(getSensor() >= kArmMaxHeight || getSensor() <= kArmMinHeight){
      motor.set(0);
    }
    motor.set(speed);
  } 

  public static double getSensor(){
    // return tenTurnPot.get();
    return motor.getSelectedSensorPosition() / (kArmDistancePerRotation * kArmGearRatio);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Bore Through", getSensor());

    // This method will be called once per scheduler run
  }
}

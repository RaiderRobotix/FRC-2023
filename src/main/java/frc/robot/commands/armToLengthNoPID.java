// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class armToLengthNoPID extends CommandBase implements Constants {
  /** Creates a new armToLengthNoPID. */
  private double length;
  private double initialValue;
  private boolean isDone = false;
  public armToLengthNoPID(double length) {
    this.length = length;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialValue = Arm.getSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(length > kArmMaxLength || length < kArmMinLength){
      end(true);
    }

    if(initialValue < length){
      if(Arm.getSensor() < length){
        Arm.setMotor(kAutoArmSpeedOut);
      } else {
        Arm.setMotor(0);
        isDone = true;
      }
    } else if (initialValue > length){
      if(Arm.getSensor() > length){
        Arm.setMotor(-kAutoArmSpeedIn);
      } else {
        Arm.setMotor(0);
        isDone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}

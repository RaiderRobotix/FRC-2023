// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class elevatorToHeightNoPID extends CommandBase implements Constants {
  /** Creates a new elevatorToHeightNoPID. */
  private double height;
  private double initialValue;
  private boolean isDone = false;
  public elevatorToHeightNoPID(double height) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialValue = Elevator.getSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(height > kElevatorMaxHeight || height < kElevatorMinHeight){
    //   end(true);
    // }

    // if(height < Elevator.getSensor()){
    //   Elevator.setMotor(kAutoElevatorSpeedUp);
    // } else if(height > Elevator.getSensor()){
    //   Elevator.setMotor(-kAutoElevatorSpeedDown);
    // } else {
    //   Elevator.setMotor(0);
    //   isDone = true;
    // }

    // if(initialValue < height){
    //   if(Elevator.getSensor() < height){
    //     Elevator.setMotor(-kAutoElevatorSpeedUp);
    //   } else {
    //     Elevator.setMotor(0);
    //     isDone = true;
    //   }
    // } else if (initialValue > height){
    //   if(Elevator.getSensor() > height){
    //     Elevator.setMotor(kAutoElevatorSpeedDown);
    //   } else {
    //     Elevator.setMotor(0);
    //     isDone = true;
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
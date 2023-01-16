// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivebase.SwerveWheelController;

public class driveDistanceNoPID extends CommandBase implements Constants {
  /** Creates a new driveDistanceNoPID. */
  SwerveWheelController controller;
  double targetDistance;
  public driveDistanceNoPID(double distance, SwerveWheelController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.targetDistance = distance;
    addRequirements(controller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(controller.getPose().getY() < targetDistance){
      controller.setSpeed(0, 0.5, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(controller.getPose().getY() - targetDistance) < 1;
  }
}

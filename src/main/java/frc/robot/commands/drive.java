// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.drivebase.SwerveWheel;
import frc.robot.subsystems.drivebase.SwerveWheelController;

public class drive extends CommandBase {
  /** Creates a new drive. */
  SwerveWheelController swerveWheelController;
  OperatorInterface oi;

  public drive(SwerveWheelController m_controller, OperatorInterface m_oInterface) {
    swerveWheelController = m_controller;
    oi = m_oInterface;
    addRequirements(swerveWheelController);
    addRequirements(oi);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveWheelController.setSpeed(oi.getLeftX(),
        oi.getLeftY(),
        oi.getRawRightJoyStick());

    if (oi.getController().getRightBumper()) {
      swerveWheelController.setHeading(swerveWheelController.getHeading() + 45.0);
    } else if (oi.getController().getLeftBumper()) {
      swerveWheelController.setHeading(swerveWheelController.getHeading() - 45.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

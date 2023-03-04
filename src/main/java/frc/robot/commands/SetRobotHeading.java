// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;


public class SetRobotHeading extends CommandBase {
  /** Creates a new SetRobotHeading. */
  private Swerve m_swerve;
  private double angle;
  private boolean isDone;
  public SetRobotHeading(Swerve swerve, double angle) {
    this.m_swerve = swerve;
    this.angle = angle;
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.setAngle(angle);
    isDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDConstants;
import frc.robot.subsystems.Swerve;

public class driveDistance extends CommandBase {
  /** Creates a new driveDistance. */
  Swerve m_swerve;
  double distance;
  Pose2d initalPose;
  boolean isDone;

  PIDController driveController;

  public driveDistance(Swerve m_swerve, double distance) {
    this.m_swerve = m_swerve;
    this.distance = distance;

    driveController = new PIDController(PIDConstants.robotDriveDistance.kp, PIDConstants.robotDriveDistance.ki, PIDConstants.robotDriveDistance.kd);
    driveController.setTolerance(PIDConstants.robotDriveDistance.tolerance);
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initalPose = m_swerve.getPose();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveController.atSetpoint()){
      isDone = true;
    } else {
      double output = driveController.calculate(m_swerve.getPose().getX(), initalPose.getX() + distance);
      m_swerve.drive(new Translation2d(output, 0), 0, false, true);
    }
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

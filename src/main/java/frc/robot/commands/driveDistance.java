// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivebase.SwerveWheelController;

public class driveDistance extends CommandBase {
  /** Creates a new driveDistance. */
  SwerveWheelController swerveWheelController;
  Translation2d newPose;
  PIDController xController = new PIDController(0, 0, 0);
  PIDController yController = new PIDController(0, 0, 0);

  public driveDistance(Translation2d newPose, SwerveWheelController swerveWheelController) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveWheelController = swerveWheelController;
    this.newPose = newPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveWheelController.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      xController.calculate(swerveWheelController.getOdometry().getPoseMeters().getX(), newPose.getX()),
      yController.calculate(swerveWheelController.getOdometry().getPoseMeters().getY(), newPose.getY()),
    0,
      swerveWheelController.getRotation2d()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}

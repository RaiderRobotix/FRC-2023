// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivebase.SwerveWheelController;

public class autonCommand extends CommandBase implements Constants{
  /** Creates a new autonCommand. */
  SwerveWheelController m_controller;
  PathPlannerTrajectory path;
  String pathName;
  public autonCommand(SwerveWheelController m_controller, String pathName) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_controller = m_controller;
    addRequirements(m_controller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    path = PathPlanner.loadPath(pathName, new PathConstraints(kPhysicalDriveMaxSpeed, kMaxAccelerationMetersPerSecondSquared));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_controller.followTrajectoryCommand(path, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

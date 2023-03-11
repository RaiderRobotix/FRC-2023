// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveAtSpeed extends CommandBase {
  /** Creates a new DriveAtSpeed. */
  private Translation2d speeds;
  private double seconds;
  private Timer timer;
  private boolean isDone;
  private Swerve m_swerve;
  public DriveAtSpeed(Swerve m_swerve, double xSpeed, double ySpeed, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speeds = new Translation2d(xSpeed * Constants.SwerveConstants.maxSpeed, ySpeed * Constants.SwerveConstants.maxSpeed);
    this.seconds = seconds;
    this.m_swerve = m_swerve;
    this.timer = new Timer();
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() <= seconds){
      m_swerve.drive(speeds, 0, false, true);
    } else {
      timer.stop();
      m_swerve.stop();
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}

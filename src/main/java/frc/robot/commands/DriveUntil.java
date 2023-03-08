// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Swerve;

public class DriveUntil extends CommandBase {
  /** Creates a new DriveUntil. */

  LimeLight m_limelight;
  Swerve m_swerve;
  double id;
  Translation2d speeds;

  boolean isDone;

  public DriveUntil(LimeLight m_limelight, Swerve m_swerve, double id, Translation2d speeds) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_limelight = m_limelight;
    this.m_swerve = m_swerve;
    this.id = id;
    this.speeds = speeds;

    addRequirements(m_limelight);
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limelight.getTv() == 1 && m_limelight.getTID() == id){
      isDone = true;
    } else {
      m_swerve.drive(speeds, 0, true, false);
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

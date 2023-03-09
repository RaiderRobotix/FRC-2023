// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PIDConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Swerve;

public class TrackToAprilTag extends CommandBase{
  /** Creates a new TrackToAprilTag. */

  LimeLight m_limelight;
  Swerve m_swerve;

  PIDController robotangle;

  public TrackToAprilTag(LimeLight m_limelight, Swerve m_swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_limelight = m_limelight;
    this.m_swerve = m_swerve;
    
    robotangle = new PIDController(PIDConstants.robotAngle.kp, PIDConstants.robotAngle.ki, PIDConstants.robotAngle.kd);

    addRequirements(m_limelight);
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SetHeading(m_swerve, m_swerve.getYaw().getDegrees() + m_limelight.getTx());
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivebase.Drivetrain;
import frc.robot.subsystems.drivebase.SwerveWheel;

public class drive extends CommandBase {
  /** Creates a new drive. */
  Drivetrain m_drivetrain;
  OperatorInterface oi;

  public drive(Drivetrain drive, OperatorInterface operatorInterface) {
    m_drivetrain = drive;
    oi = operatorInterface;
    addRequirements(m_drivetrain, oi);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(Gyro.gyro().getAngle());
    // System.out.println(oi.getLeftY());
    // System.out.println(oi.getLeftX());
    // System.out.println(oi.getRightX());
    Drivetrain.drive(oi.getLeftY(), oi.getLeftX(), oi.getRightX());
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

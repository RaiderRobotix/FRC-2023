// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.OperatorInterface;

public class DefaultDriveBaseCommand extends CommandBase {
  /** Creates a new DefaultDriveBaseCommand. */

  DriveBase drives;
  private final OperatorInterface oi;

  public DefaultDriveBaseCommand(DriveBase m_DriveBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    drives = m_DriveBase;
    oi = OperatorInterface.getInstance();

    addRequirements(drives);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drives.setSpeed(oi.getLeftY(), oi.getRightY());
    // System.out.println(drives.leftFrontMotor.getSelectedSensorVelocity());
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

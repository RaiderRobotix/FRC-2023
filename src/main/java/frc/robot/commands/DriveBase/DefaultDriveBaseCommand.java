// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import frc.robot.subsystems.DriveBase.DriveTrain;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.DriveBase.DriveTrain;


/** An example command that uses an example subsystem. */
public class DefaultDriveBaseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  DriveTrain driveTrain;
  OperatorInterface oi;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultDriveBaseCommand(DriveTrain m_driveTrain, OperatorInterface m_oi) {
    driveTrain = m_driveTrain;
    oi = m_oi;
    

    // Use addRequirements(z) here to declare subsystem dependencies.
    addRequirements(driveTrain);
    addRequirements(oi);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setSpeed(-0.1 * oi.getLeftY(), -0.1 * oi.getRightY());


  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeed(0);
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.Pneumatics;

import javax.swing.text.PlainView;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PneumaticsCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Pneumatics m_PneumaticsSub;
  private OperatorInterface oi;
  private double PSI;
  boolean state;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PneumaticsCmd(Pneumatics Pneumatics,OperatorInterface m_oInterface, boolean state) {
    m_PneumaticsSub = Pneumatics;
    oi = m_oInterface;
    this.state = state;


    // Use addRequirements(z) here to declare subsystem dependencies.
    addRequirements(oi);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_PneumaticsSub.setSolenoidValue(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_PneumaticsSub.setSolenoidValue(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
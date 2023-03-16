// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToHumanPlayerPreset extends SequentialCommandGroup {
  /** Creates a new ToLowRowPreset. */
  Elevator m_elevator;
  Arm m_arm;
  public ToHumanPlayerPreset(Elevator elevator, Arm arm) {
    m_elevator = elevator;
    m_arm = arm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorToHeight(m_elevator, Constants.Elevator.humanPlayerHeight),
      new ArmToPosition(m_arm, Constants.Arm.humanPlayerLength));  }
}

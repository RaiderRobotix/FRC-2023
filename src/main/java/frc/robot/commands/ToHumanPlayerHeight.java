// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToHumanPlayerHeight extends ParallelCommandGroup {
  /** Creates a new ToHumanPlayerHeight. */
  public ToHumanPlayerHeight(Elevator m_elevator, Arm m_arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorToHeight(m_elevator, Constants.Elevator.humanPlayerHeight),
      new ArmToPosition(m_arm, Constants.Arm.humanPlayerLength)
    );
  }
}

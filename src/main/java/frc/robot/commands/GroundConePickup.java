// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundConePickup extends SequentialCommandGroup {
  /** Creates a new GroundConePickup. */
  public GroundConePickup(Swerve m_swerve, Arm m_arm, Grabber m_grabber, Elevator m_elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new ElevatorToHeight(m_elevator, Constants.Elevator.lowRowHeight)
      new GroundPickup(m_swerve, m_arm),
      new InstantCommand(() -> m_grabber.closeGrabber()),
      new ToHumanPlayerHeight(m_elevator, m_arm)

    );
  }
}

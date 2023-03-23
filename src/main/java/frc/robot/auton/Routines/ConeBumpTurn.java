// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.Routines;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeBumpTurn extends SequentialCommandGroup {
  /** Creates a new ConeBumpTurn. */
  public ConeBumpTurn(Swerve m_swerve, Elevator m_elevator, Arm m_arm, Grabber m_grabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorToHeight(m_elevator, Constants.Elevator.topRowHeight),
      new ArmToPosition(m_arm, Constants.Arm.topRowLength),
      new InstantCommand(() -> m_grabber.openGrabber()),
      new InstantCommand(() -> m_swerve.setAngle(0)),
      new ArmToPosition(m_arm, Constants.Arm.humanPlayerLength),
      new DriveAtSpeed(m_swerve, 0.1, 0, 0.3),
      new WaitCommand(0.5),
      new DriveAtSpeed(m_swerve, -0.2, 0, 6.0),
      new WaitCommand(0.5),
      new SetHeading(m_swerve, 180)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.Routines;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.balance;
import frc.robot.commands.DriveAtSpeed;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.ElevatorToHeight;
import frc.robot.commands.SetRobotHeading;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BumpSideGrabAuto extends SequentialCommandGroup {
  /** Creates a new simpleAuto. */
  public BumpSideGrabAuto(Swerve m_swerve, Pneumatics m_pneumatics, Arm m_arm, Elevator m_elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_swerve.zeroGyro()),
      new InstantCommand(() -> m_pneumatics.popPopper()),
      new WaitCommand(5),
      new DriveAtSpeed(m_swerve, 0.1, 0.1, 0.3), // move up and left to clear bridge
      new WaitCommand(0.5),
      new InstantCommand(() -> m_swerve.setAngle(0)),
      new WaitCommand(0.5),
      new DriveAtSpeed(m_swerve, 0.2, 0, 6.0),
      new WaitCommand(0.5),
      new ElevatorToHeight(m_elevator, Constants.Elevator.humanPlayerHeight),
      new ArmToPosition(m_arm, Constants.Arm.humanPlayerLength));
      //new DriveAtSpeed(m_swerve, 0.0, 0.4, 0.6), // Slams into left wall
      // new WaitCommand(1));
     // new ArmToPosition(m_arm, Constants.Arm.floorPickupLength));
  }
}

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
import frc.robot.auton.Routines.DriveBackRaiseArm;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoBumpSideGrabAuto extends SequentialCommandGroup {
  /** Creates a new simpleAuto. */
  public NoBumpSideGrabAuto(Swerve m_swerve, Pneumatics m_pneumatics, Arm m_arm, Elevator m_elevator, Grabber m_grabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_swerve.zeroGyro()),
      new InstantCommand(() -> m_pneumatics.popPopper()),
      new WaitCommand(1),
      new DriveAtSpeed(m_swerve, 0.1, -0.1, 0.5),
      new WaitCommand(0.5),
      new DriveAtSpeed(m_swerve, 0.2, 0, 6),
      new WaitCommand(0.5),
      new ArmToPosition(m_arm, Constants.Arm.floorPickupLength),
      new InstantCommand(() -> m_grabber.closeIfObjectDetected()),
      new WaitCommand(0.5),
      new DriveBackRaiseArm(m_swerve, m_arm, m_elevator, m_grabber, 0.4, 2));
  }
}

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
import frc.robot.commands.DriveDistance;
import frc.robot.commands.SetRobotHeading;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;
import frc.robot.auton.Routines.CrossBridgeBalance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PopCrossBalance extends SequentialCommandGroup {
  /** Creates a new simpleAuto. */
  public PopCrossBalance(Swerve m_swerve, Pneumatics m_pneumatics) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_swerve.zeroGyro()),
      new InstantCommand(() -> m_pneumatics.popPopper()),
      new WaitCommand(1),
      new CrossBridgeBalance(m_swerve));
  }
}

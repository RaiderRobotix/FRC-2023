// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.Routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test90 extends SequentialCommandGroup {
  /** Creates a new Test90. */
  private String[] paths = {
    "90 Degree 0",
    "90 Degree 1"
  };

  public Test90(Swerve m_swerve, Pneumatics m_pneumatics) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new AutoFromPathPlanner(m_swerve, paths[0], 5, true),
      new InstantCommand(() -> m_pneumatics.popPopper()),
      new AutoFromPathPlanner(m_swerve, paths[1], 5, true)
    );
  }
}

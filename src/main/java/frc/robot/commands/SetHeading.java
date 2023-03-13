// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.PIDConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetHeading extends PIDCommand {
  /** Creates a new SetHeading. */
  public SetHeading(Swerve m_swerve, double desiredHeading) {
    super(
        // The controller that the command will use
        new PIDController(PIDConstants.robotAngle.kp, PIDConstants.robotAngle.ki, PIDConstants.robotAngle.kp),
        // This should return the measurement
        () -> m_swerve.getRotation2dIEEE().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> desiredHeading,
        // This uses the output
        output -> {
          m_swerve.drive(new Translation2d(0,0), output, false, true);
        });
    addRequirements(m_swerve);
    getController().setTolerance(PIDConstants.robotAngle.tolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

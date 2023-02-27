// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.drivebase.SwerveWheelController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class turnToAngle extends PIDCommand implements Constants {
  /** Creates a new turnToAngle. */
  public turnToAngle(double angle, SwerveWheelController swerveController) {
    super(
        // The controller that the command will use
        new PIDController(robotangleKp, robotangleKi, robotangleKd),
        // This should return the measurement
        () -> Gyro.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> swerveController.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, output, swerveController.getRotation2d())));
          // Use the output here
        addRequirements(swerveController);
        getController().setTolerance(robotAngleTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
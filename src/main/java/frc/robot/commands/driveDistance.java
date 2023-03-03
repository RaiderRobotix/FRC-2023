// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.UniqueConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class driveDistance extends PIDCommand implements UniqueConstants {
  /** Creates a new driveDistance. */
  //Distance is measured in 
  public driveDistance(double distance, Swerve swerveController) {
    super(
        // The controller that the command will use
        new PIDController(robotDriveDistanceKp, robotDriveDistanceKi, robotDriveDistanceKd),
        // This should return the measurement
        () -> swerveController.getPose().getX(),
        // This should return the setpoint (can also be a constant)
        distance + swerveController.getPose().getX(),
        // This uses the outputs
        output -> swerveController.drive(new Translation2d(distance, 0), 0, true, true));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveController);
    getController().setTolerance(robotDistanceTolerance);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
} 

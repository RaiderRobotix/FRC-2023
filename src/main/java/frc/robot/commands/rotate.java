// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.drivebase.SwerveWheelController;

public class rotate extends CommandBase {
  /** Creates a new rotate. */

  SwerveWheelController swerveWheelController;
  OperatorInterface oi;

  private double angle;
  private boolean reverse;
  private boolean change;

  public rotate(SwerveWheelController swerveWheelController, OperatorInterface m_OperatorInterface, double angle,
      boolean change) {
    this.swerveWheelController = swerveWheelController;
    oi = m_OperatorInterface;
    addRequirements(swerveWheelController);
    addRequirements(oi);

    this.reverse = false;

    this.angle = angle;

    this.change = change;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (change) {
      angle += Gyro.getHeading();
      if (angle > 180) {
        angle -= 360;
      } else if (angle < -180) {
        angle += 360;
      }
    }

    if (angle - Gyro.getHeading() >= 0) {
      reverse = false;
    } else {
      reverse = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (Math.abs((Gyro.getHeading() - angle)) >= .5) {
      // System.out.println(Math.abs((Gyro.getHeading() - angle)));
      SmartDashboard.putNumber("Rotate Angle", angle);
      System.out.println("Angle: " + angle + " " + Gyro.getHeading());
      if (oi.getController().getAButton()) {
        end(true);
        break;
      }
      if (!reverse) {
        swerveWheelController.setSpeed(0, 0, Constants.rotateSpeed);
      } else {
        swerveWheelController.setSpeed(0, 0, -Constants.rotateSpeed);
      }
    }
    end(false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveWheelController.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

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
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.drivebase.drivebaseConstants;
import frc.robot.subsystems.drivebase.SwerveWheelController;

public class rotate extends CommandBase {
  /** Creates a new rotate. */

  SwerveWheelController swerveWheelController;
  OperatorInterface oi;
  double angle;
  boolean reverse;

  public rotate(SwerveWheelController swerveWheelController, OperatorInterface m_OperatorInterface, double angle,
      boolean reverse) {
    this.swerveWheelController = swerveWheelController;
    oi = m_OperatorInterface;
    addRequirements(swerveWheelController);
    addRequirements(oi);
    this.angle = angle;
    this.reverse = reverse;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TrajectoryConfig trajectoryConfig = new
    // TrajectoryConfig(drivebaseConstants.kPhysicalDriveMaxSpeed,
    // drivebaseConstants.kPhysicalSteerMaxSpeed).setKinematics(swerveWheelController.kinematics);
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0, 0, Rotation2d.fromDegrees(Gyro.gyro().getYaw())),
    // List.of(
    // new Translation2d(0, 0)),
    // new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
    // trajectoryConfig);

    // PIDController xController = new PIDController(1, 0, 0);
    // PIDController yController = new PIDController(1, 0, 0);
    // ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0,
    // drivebaseConstants.kThetaControllerConstraint);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // trajectory,
    // swerveWheelController::getPose,
    // swerveWheelController.kinematics,
    // xController,
    // yController,
    // thetaController,
    // swerveWheelController::setState,
    // swerveWheelController);

    // return new SequentialCommandGroup(
    // new InstantCommand(() ->
    // swerveWheelController.resetOdometry(trajectory.getInitialPose())),
    // swerveControllerCommand,
    // new InstantCommand(() -> swerveWheelController.stopMotors()));

    // System.out.println("Passed");

    // if (angle > 360) {
    // angle -= 360;
    // }
    // if (angle < 0) {
    // angle = +360;
    // }

    System.out.println("Angle: " + angle + " Yaw " + Gyro.getHeading());
    //Make Reverse dependent on which value it is closer too
    while (Math.abs(Gyro.getHeading() - angle) > 2) {
      if (!reverse) {
        swerveWheelController.setSpeed(0, 0, drivebaseConstants.rotateSpeed, drivebaseConstants.rotateSpeed);
      } else {
        swerveWheelController.setSpeed(0, 0, -drivebaseConstants.rotateSpeed, drivebaseConstants.rotateSpeed);
      }
      SmartDashboard.updateValues();
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

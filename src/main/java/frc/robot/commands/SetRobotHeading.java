// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.Arrays;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;


public class SetRobotHeading extends CommandBase {
  /** Creates a new SetRobotHeading. */
  private Swerve m_swerve;
  private double desiredAngle;
  private boolean isDone;
  private double angle;
  public SetRobotHeading(Swerve swerve, double angle) {
    this.m_swerve = swerve;
    this.angle = angle;
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = m_swerve.getYawIEEE();
    double alpha = desiredAngle - currentAngle;
    double beta = desiredAngle - currentAngle + 360;
    double gamma = desiredAngle - currentAngle - 360;
    //double shortRotationAngle = (desiredAngle - currentAngle + 540) % 360 - 180;



    if(Math.abs(currentAngle - m_swerve.getYawIEEE()) <= Constants.SwerveConstants.setHeadingTolerence){
      if(gamma > 0){
       m_swerve.drive(new Translation2d(0,0), Constants.SwerveConstants.rotateSpeed, false, false);
      } else {
        m_swerve.drive(new Translation2d(0,0), -Constants.SwerveConstants.rotateSpeed, false, false);
      }
    } else  {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}

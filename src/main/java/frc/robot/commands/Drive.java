// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.DriveBase.SwerveWheel;
import frc.robot.subsystems.DriveBase.SwerveWheelController;

public class Drive extends CommandBase {
  /** Creates a new drive. */
  SwerveWheelController swerveWheelController;
  OperatorInterface oi;
  private double maxSpeed;

  public Drive(SwerveWheelController m_controller, OperatorInterface m_oInterface, double maxSpeed) {
    swerveWheelController = m_controller;
    oi = m_oInterface;
    this.maxSpeed = maxSpeed;
    addRequirements(swerveWheelController);
    addRequirements(oi);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SwerveWheelController.resetMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveWheelController.setSpeed(
      oi.getLeftX()*maxSpeed,
      oi.getLeftY()*maxSpeed,
      oi.getRightY()*maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
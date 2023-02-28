// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.drivebase.SwerveWheelController;

public class drive extends CommandBase implements Constants {
  /** Creates a new drive. */
  SwerveWheelController swerveWheelController;
  OperatorInterface oi;
  private double maxSpeed;

  public drive(SwerveWheelController m_controller, OperatorInterface m_oInterface, double maxSpeed) {
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
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  @Override
  public void execute() {
    if(oi.getLeftTrigger()){
      maxSpeed = 0.2;
    } else if(oi.getRightTrigger()){
      maxSpeed = 1.0;
    } else {
      maxSpeed = 0.6;
    }
    swerveWheelController.drive(ChassisSpeeds
          .fromFieldRelativeSpeeds(
            oi.getLeftY() * maxAttainableSpeed * maxSpeed,
            oi.getLeftX() * maxAttainableSpeed * maxSpeed,
            -oi.getRightX() * 5,
            Rotation2d.fromDegrees(Gyro.getHeading())));
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
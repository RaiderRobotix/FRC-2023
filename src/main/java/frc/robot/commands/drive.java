// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.drivebase.SwerveWheel;
import frc.robot.subsystems.drivebase.SwerveWheelController;

public class drive extends CommandBase {
  /** Creates a new drive. */
  SwerveWheelController swerveWheelController;
  OperatorInterface oi;

  public drive(SwerveWheelController m_controller, OperatorInterface m_oInterface) {
    swerveWheelController = m_controller;
    oi = m_oInterface;
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
    if(oi.getController().getAButton()){
      switch (oi.getController().getPOV()) {
        case 90:
          swerveWheelController.setSpeed(0, 1, 0, Constants.kPhysicalDriveMaxSpeed);
        case 180:
          swerveWheelController.setSpeed(-1, 0, 0, Constants.kPhysicalDriveMaxSpeed);
        case 270:
          swerveWheelController.setSpeed(0, -1, 0, Constants.kPhysicalDriveMaxSpeed);
        case 0:
          swerveWheelController.setSpeed(1, 0, 0, Constants.kPhysicalDriveMaxSpeed);
        default:
          swerveWheelController.stopMotors();
      }} else if (oi.getController().getLeftTriggerAxis() > 0.75) {
        if (oi.getController().getAButton()) {
        } else {
          swerveWheelController.setSpeed(oi.getLeftY(),
              oi.getLeftX(),
              -1 * oi.getRightX(),
              0.2);
        }
    } else if (oi.getController().getRightTriggerAxis() > 0.75) {
      switch (oi.getController().getPOV()) {
        case 0:
          swerveWheelController.setAngle(0);
          break;
        case 90:
          swerveWheelController.setAngle(90);
          break;
        case 180:
          swerveWheelController.setAngle(180);
          break;
        case 270:
          swerveWheelController.setAngle(270);
          break;
        default:
          break;
      }
      swerveWheelController.setSpeed(oi.getLeftY(),
          oi.getLeftX(),
          -1 * oi.getRightX(),
          Constants.kPhysicalDriveMaxSpeed);

      
    } else {
      swerveWheelController.setSpeed(oi.getLeftY(),
          oi.getLeftX(),
          -1 * oi.getRightX(),
          0.6);
    }
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

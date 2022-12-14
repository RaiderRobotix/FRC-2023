// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class OperatorInterface extends SubsystemBase implements Constants {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final XboxController controller;

  public OperatorInterface() {
    this.controller = new XboxController(xboxControllerPort);
  }

  public double getRawRightJoyStick() {
    return Math.atan2(getRightY(), getRightX());
  }

  public double getRawLeftJoyStick() {
    return Math.atan2(getLeftY(), getLeftX());
  }

  public XboxController getController() {
    return this.controller;
  }

  public double getLeftY() {
    if (Math.abs(controller.getLeftY()) > leftDeadband) {
      return -1 * controller.getLeftY();
    } else {
      return 0;
    }
  }

  public double getLeftX() {
    if (Math.abs(controller.getLeftX()) > leftDeadband) {
      return -1 * controller.getLeftX();
    } else {
      return 0;
    }
  }

  public double getRightY() {
    if (Math.abs(controller.getRightY()) > rightDeadband) {
      return controller.getRightY();
    } else {
      return 0;
    }
  }

  public double getRightX() {
    if (Math.abs(controller.getRightX()) > rightDeadband) {
      return controller.getRightX();
    } else {
      return 0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // @Override
  // public void initDefaultCommand() {
  // // Set the default command for a subsystem here.
  // // setDefaultCommand(new MySpecialCommand());
  // }
}

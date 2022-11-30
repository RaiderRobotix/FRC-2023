// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
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
    return Math.atan2(controller.getRightY(), controller.getRightX());
  }

  public XboxController getController() {
    return controller;
  }

  // @Override
  // public void initDefaultCommand() {
  // // Set the default command for a subsystem here.
  // // setDefaultCommand(new MySpecialCommand());
  // }
}

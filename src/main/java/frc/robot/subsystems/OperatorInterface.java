// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class OperatorInterface extends SubsystemBase implements Constants {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final XboxController xboxcontroller;
  private final Joystick operatorStick;

  public OperatorInterface() {
    this.xboxcontroller = new XboxController(xboxControllerPort);
    this.operatorStick = new Joystick(operatorStickPort);
  }

  public double getRawRightJoyStick() {
    return Math.atan2(getRightY(), getRightX());
  }

  public double getRawLeftJoyStick() {
    return Math.atan2(getLeftY(), getLeftX());
  }

  public XboxController getXboxController() {
    return this.xboxcontroller;
  }

  public Joystick getOperatorStick(){
    return this.operatorStick;
  }


  public double getLeftY() {
    if (Math.abs(xboxcontroller.getLeftY()) > leftDeadband) {
      return -1 * xboxcontroller.getLeftY();
    } else {
      return 0;
    }
  }

  public double getLeftX() {
    if (Math.abs(xboxcontroller.getLeftX()) > leftDeadband) {
      return -1 * xboxcontroller.getLeftX();
    } else {
      return 0;
    }
  }

  public double getRightY() {
    if (Math.abs(xboxcontroller.getRightY()) > rightDeadband) {
      return xboxcontroller.getRightY();
    } else {
      return 0;
    }
  }

  public double getRightX() {
    if (Math.abs(xboxcontroller.getRightX()) > rightDeadband) {
      return xboxcontroller.getRightX();
    } else {
      return 0;
    }
  }

  public boolean getRightTrigger(){
    return xboxcontroller.getRightTriggerAxis() > rightTriggerThreshold;
  }

  public boolean getLeftTrigger(){
    return xboxcontroller.getLeftTriggerAxis() > leftTriggerThreshold;
  }

  public boolean isPOV(){
    if(xboxcontroller.getPOV() == -1){
      return false;
    } else {
      return true;
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
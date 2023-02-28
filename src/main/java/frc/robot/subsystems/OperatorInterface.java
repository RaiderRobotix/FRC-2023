// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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
  private final Joystick operatorJoystick;
  private final DigitalInput sensor;

  public OperatorInterface() {
    this.xboxcontroller = new XboxController(xboxControllerPort);
    this.operatorJoystick = new Joystick(operatorStickPort);
    this.sensor = new DigitalInput(kGrabberDistanceSensorDIO);
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

  public Joystick getOperatorJoystick(){
    return this.operatorJoystick;
  }

  public boolean getDistanceSensor(){
    return !sensor.get();
  }


  public double getLeftY() {
    if (Math.abs(xboxcontroller.getLeftY()) > leftDeadband) {
      return Math.pow(-xboxcontroller.getLeftY(), 3);
    } else {
      return 0;
    }
  }

  public double getLeftX() {
    if (Math.abs(xboxcontroller.getLeftX()) > leftDeadband) {
      return Math.pow(-xboxcontroller.getLeftX(), 3);
    } else {
      return 0;
    }
  }

  public double getRightY() {
    if (Math.abs(xboxcontroller.getRightY()) > rightDeadband) {
      return Math.pow(xboxcontroller.getRightY(), 3);
    } else {
      return 0;
    }
  }

  public double getRightX() {
    if (Math.abs(xboxcontroller.getRightX()) > rightDeadband) {
      return Math.pow(xboxcontroller.getRightX(), 3);
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
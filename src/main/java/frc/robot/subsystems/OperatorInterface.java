// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorInterface extends SubsystemBase {
  /** Creates a new OperatorInterface. */

  private static OperatorInterface m_instance;

  private XboxController controller;

  private final Joystick leftStick;
  private final Joystick rightStick;

  public OperatorInterface() {
    this.leftStick = new Joystick(0);
    this.rightStick = new Joystick(1);

    this.controller = new XboxController(0);
  }

  public double getLeftY() {
    double tmp = leftStick.getY();
    // double tmp = controller.getLeftY();
    if (Math.abs(tmp) > .15) {
      return tmp;
    }
    return 0.0;
  }

  public double getRightY() {
    double tmp = rightStick.getY();
    // double tmp = controller.getRightY();
    // System.out.println(tmp);
    if (Math.abs(tmp) > .15) {
      // System.out.println("passed");
      return tmp;
    }
    return 0.0;
  }

  public boolean getLeftButton(int btn) {
    if (leftStick.getRawButton(btn)) {
      return true;
    }
    return false;
  }

  public static OperatorInterface getInstance() {
    System.out.println("x");
    if (m_instance == null) {
      m_instance = new OperatorInterface();
    }
    return m_instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

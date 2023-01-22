// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.DriveBase.SwerveWheelController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveWheelController m_controller = new SwerveWheelController();
  private final OperatorInterface m_operatorInterface = new OperatorInterface();

  private final Gyro m_gyro = new Gyro();

  private final drive m_Drive = new drive(m_controller, m_operatorInterface, 0.6);

  private final drive m_autoCommand = m_Drive;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    this.m_controller.setDefaultCommand(m_Drive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(m_operatorInterface::getLeftTrigger)
      .onTrue(new drive(m_controller, m_operatorInterface, 0.2));

    new Trigger(m_operatorInterface::getLeftTrigger)
      .onTrue(new drive(m_controller, m_operatorInterface, 1));

    new JoystickButton(m_operatorInterface.getController(), XboxController.Button.kBack.value)
      .onTrue(new InstantCommand(() -> SwerveWheelController.toggleCoast()));

    new JoystickButton(m_operatorInterface.getController(), XboxController.Button.kStart.value)
      .onTrue(new InstantCommand(() -> SwerveWheelController.toggleField()));

    new JoystickButton(m_operatorInterface.getController(), XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> SwerveWheelController.reset()));

    new JoystickButton(m_operatorInterface.getController(), XboxController.Button.kA.value)
      .and(new Trigger(m_operatorInterface::isPOV)
      .onTrue(new StartEndCommand(
        () -> m_controller.setAngle(m_operatorInterface.getController().getPOV()),
        () -> m_controller.setAngle(0))));

    new JoystickButton(m_operatorInterface.getController(), XboxController.Button.kB.value)
      .and(new Trigger(m_operatorInterface::isPOV)
      .onTrue(new StartEndCommand(
        () -> m_controller.setSpeed(Math.cos(m_operatorInterface.getController().getPOV()), Math.sin(m_operatorInterface.getController().getPOV()), 0),
        () -> m_controller.stopMotors())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
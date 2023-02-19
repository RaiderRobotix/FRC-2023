// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutonCommands;
import frc.robot.auto.AutonSelector;
import frc.robot.commands.armToLength;
import frc.robot.commands.drive;
import frc.robot.commands.WIPdriveDistance;
import frc.robot.commands.elevatorToHeight;
import frc.robot.commands.motor;
import frc.robot.commands.pickdown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.drivebase.SwerveWheelController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Constants {
  // The robot's subsystems and commands are defined here...
  private final SwerveWheelController m_controller = new SwerveWheelController();
  private final OperatorInterface m_operatorInterface = new OperatorInterface();
  private final Pneumatics mPneumatics = new Pneumatics(m_operatorInterface);
  private final Gyro m_gyro = new Gyro();
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  private final AutonSelector autonSelector = new AutonSelector();

  private final WIPdriveDistance m_autoCommand = new WIPdriveDistance(1,m_controller);

  private final drive m_Drive = new drive( m_controller, m_operatorInterface, 0.6);

  private motor m_motor = new motor(m_operatorInterface);

  // private DigitalInput sensor;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    this.m_controller.setDefaultCommand(m_Drive);
    // sensor = new DigitalInput(0);
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
    // Driver Controls
    // new Trigger(m_operatorInterface::getLeftTrigger)
    //   .whileTrue(new drive(m_controller, m_operatorInterface, slowSpeed));

    // new Trigger(m_operatorInterface::getRightTrigger)
    //   .whileTrue(new drive(m_controller, m_operatorInterface, turboSpeed));

    new JoystickButton(m_operatorInterface.getXboxController(), XboxController.Button.kBack.value)
      .onTrue(new InstantCommand(() -> SwerveWheelController.toggleCoast()));

    new JoystickButton(m_operatorInterface.getXboxController(), XboxController.Button.kStart.value)
      .onTrue(new InstantCommand(() -> SwerveWheelController.toggleField()));

    new JoystickButton(m_operatorInterface.getXboxController(), XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> SwerveWheelController.reset()));

    new JoystickButton(m_operatorInterface.getXboxController(), XboxController.Button.kA.value)
      .and(new Trigger(m_operatorInterface::isPOV))
      .whileTrue(new StartEndCommand(
        () -> m_controller.setAngle(m_operatorInterface.getXboxController().getPOV()),
        () -> m_controller.setAngle(0)));
      
    new JoystickButton(m_operatorInterface.getXboxController(), XboxController.Button.kB.value)
      .and(new Trigger(m_operatorInterface::isPOV)
      .whileTrue(new StartEndCommand(
        () -> m_controller.setSpeed(Math.cos(m_operatorInterface.getXboxController().getPOV()), Math.sin(m_operatorInterface.getXboxController().getPOV()), 0),
        () -> m_controller.stopMotors(),
        m_controller)));
        
      // Operator Controls
    new JoystickButton(m_operatorInterface.getOperatorJoystick(), grabberJoystickButton)
      .and(new Trigger(Arm::isUpperRow).negate())
      .onTrue(new InstantCommand(() -> Pneumatics.toggleGrabberSolenoid()));

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), grabberJoystickButton)
    .onTrue(new pickdown());

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), popperJoystickButton)
    .onTrue(new InstantCommand(() -> Pneumatics.togglePopperSolenoid()));

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), armInJoystickButton)
      .and(new Trigger(Arm::getSensorLow).negate())
      .whileTrue(new StartEndCommand(
        () -> Arm.setMotor(-kArmInSpeed),
        () -> Arm.setMotor(0)));

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), armOutJoystickButton)
      .and(new Trigger(Arm::getSensorMax).negate())
      .whileTrue(new StartEndCommand(
        () -> Arm.setMotor(kArmOutSpeed),
        () -> Arm.setMotor(0),
        m_arm));

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), elevatorUpJoystickButton)
    .and(new Trigger(Elevator::getSensorMax).negate())
    .whileTrue(new StartEndCommand(
      () -> Elevator.setMotor(-kElevatorUpSpeed),
      () -> Elevator.setMotor(0),
      m_elevator));
      
    new JoystickButton(m_operatorInterface.getOperatorJoystick(), elevatorDownJoystickButton)
    .and(new Trigger(Elevator::getSensorLow).negate())
    .whileTrue(new StartEndCommand(
      () -> Elevator.setMotor(kElevatorDownSpeed),
      () -> Elevator.setMotor(0),
      m_elevator));

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), elevatorLowerRowJoystickButton)
    .whileTrue(new armToLength(kFloorLength))
    .whileTrue(new elevatorToHeight(kLowerRowHeight));

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), elevatorMidRowJoystickButton)
    .whileTrue(new armToLength(kMidRowLength))
    .whileTrue(new elevatorToHeight(kMidRowHeight));

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), elevatorUpperRowJoystickButton)
    .whileTrue(new armToLength(kUpperRowLength))
    .whileTrue(new elevatorToHeight(kUpperRowHeight));

    new JoystickButton(m_operatorInterface.getOperatorJoystick(), elevatorHumanPlayerJoystickButton)
      .whileTrue(new armToLength(kHumanPlayerLength))
      .whileTrue(new elevatorToHeight(kHumanPlayerHeight));





    //Button for Sensor trigger
    new Trigger(m_operatorInterface::getDistanceSensor)
      .and(new JoystickButton(m_operatorInterface.getOperatorJoystick(), autoGrabberJoystickButton))
      // .debounce(kDistanceSensorDebounceTime)
      .onTrue(new pickdown());
    
    // new Trigger(Arm::isUpperRow).negate()
    // .onTrue(new armToLength(kHumanPlayerLength))
    // .onTrue(new elevatorToHeight(kHumanPlayerHeight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutonCommands command = autonSelector.getCommand(m_controller);
    Map<String, Command> eventMap = command.getEventMap();

    SwerveAutoBuilder autoBuilder =  new SwerveAutoBuilder(
      m_controller::getPose, // Pose2d supplier
      SwerveWheelController::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of autokDriveKinematics, // SwerveDriveKinematics
      kDriveKinematics,
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      m_controller::setState, // Module states consumer used to output to the drive subsystemeventMap,
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      m_controller // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(command.getPath());
  }
}
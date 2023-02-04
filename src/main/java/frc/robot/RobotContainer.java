// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import frc.robot.commands.balance;
import frc.robot.commands.drive;
import frc.robot.commands.driveDistance;
import frc.robot.commands.motor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.drivebase.SwerveWheel;
import frc.robot.subsystems.drivebase.SwerveWheelController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  private final driveDistance m_autoCommand = new driveDistance(1,m_controller);

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
    new Trigger(m_operatorInterface::getRightTrigger)
      .onTrue(new drive(m_controller, m_operatorInterface, slowSpeed));

    new Trigger(m_operatorInterface::getLeftTrigger)
      .onTrue(new drive(m_controller, m_operatorInterface, turboSpeed));

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
      () -> m_controller.stopMotors())));
        
      // Operator Controls
      new JoystickButton(m_operatorInterface.getOperatorJoystick(), grabberJoystickButton)
        .onTrue(new InstantCommand(() -> Pneumatics.toggleGrabberSolenoid()));
      // new JoystickButton(m_operatorInterface.getOperatorJoystick(), 2)
      //   .onTrue(new InstantCommand(() -> Pneumatics.setGrabberSolenoid(!sensor.get())));
      new JoystickButton(m_operatorInterface.getOperatorJoystick(), 3)
        .whileTrue(m_motor);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                kPhysicalDriveMaxSpeed,
                kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0,0, new Rotation2d().fromDegrees(0)),
                    // new Pose2d(2,0, new Rotation2d().fromDegrees(90)),
                    new Pose2d(5, 0, new Rotation2d().fromDegrees(0))
            ),
            config);

    var thetaController =
        new ProfiledPIDController(
            thetaControllerKp, thetaControllerKi, thetaControllerKd, kThetaControllerConstraint);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_controller::getPose, // Functional interface to feed supplier
            kDriveKinematics,
            // Position controllers
            new PIDController(xControllerKp, xControllerKi, xControllerKd),
            new PIDController(yControllerKp, yControllerKi, yControllerKd),
            thetaController,
            () -> Rotation2d.fromDegrees(180),
            m_controller::setState,
            m_controller);

    // Reset odometry to the starting pose of the trajectory.
    m_controller.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_controller.setSpeed(0, 0, 0, true));
  }
}
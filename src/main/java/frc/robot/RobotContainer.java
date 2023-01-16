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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drive;
import frc.robot.commands.driveDistance;
import frc.robot.commands.driveDistanceNoPID;
import frc.robot.commands.rotate;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.drivebase.SwerveWheel;
import frc.robot.subsystems.drivebase.SwerveWheelController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveWheelController m_controller = new SwerveWheelController();
  private final OperatorInterface m_operatorInterface = new OperatorInterface();

  private final Gyro m_gyro = new Gyro();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final drive m_Drive = new drive(m_controller, m_operatorInterface);
  private final rotate m_rotate = new rotate(m_controller, m_operatorInterface, 0, false);

  private final Button leftBumber = new JoystickButton(m_operatorInterface.getController(), Constants.leftBumberId);
  private final Button rightBumber = new JoystickButton(m_operatorInterface.getController(), Constants.rightBumberId);
  private final Button xButton = new JoystickButton(m_operatorInterface.getController(), 2);

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
    new JoystickButton(m_operatorInterface.getController(), XboxController.Button.kBack.value).onTrue(new InstantCommand(() -> SwerveWheelController.toggleCoast()));
    new JoystickButton(m_operatorInterface.getController(), XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> SwerveWheelController.toggleField()));
    new JoystickButton(m_operatorInterface.getController(), XboxController.Button.kY.value).onTrue(new InstantCommand(() -> SwerveWheelController.reset()));
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
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
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
            m_controller::setState,
            m_controller);

    // Reset odometry to the starting pose of the trajectory.
    m_controller.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_controller.setSpeed(0, 0, 0, true));
  }
}
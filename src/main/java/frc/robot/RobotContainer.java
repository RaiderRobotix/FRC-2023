package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Operator Buttons */
    private final JoystickButton elevatorUpButton = new JoystickButton(operator, 5);
    private final JoystickButton elevatorDownButton = new JoystickButton(operator, 3);

    private final JoystickButton autoScoreHighButton = new JoystickButton(operator, 8);
    private final JoystickButton autoScoreMidButton = new JoystickButton(operator, 10);
    private final JoystickButton autoLowButton = new JoystickButton(operator, 12);
    private final JoystickButton autoHPButton = new JoystickButton(operator, 7);
    
    private final JoystickButton armOutButton = new JoystickButton(operator, 9);
    private final JoystickButton armInButton = new JoystickButton(operator, 11);

    /* Subsystems */
    private final Swerve m_swerve = new Swerve();
    // private final Arm m_arm = new Arm();
    // private final Elevator m_elevator = new Elevator();
    // private final Pneumatics m_pneumatics = new Pneumatics();
    // private final Grabber m_grabber = new Grabber();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                m_swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));

        /* Operator Buttons */
        // armOutButton.whileTrue(
        //     new StartEndCommand(
        //         () -> m_arm.extend(Constants.Arm.manualSpeed),
        //         () -> m_arm.stop(),
        //         m_arm)
        // );

        // armInButton.whileTrue(
        //     new StartEndCommand(
        //         () -> m_arm.retract(Constants.Arm.manualSpeed),
        //         () -> m_arm.stop(),
        //         m_arm)
        // );

        // elevatorUpButton.whileTrue(
        //     new StartEndCommand(
        //         () -> m_elevator.moveUp(Constants.Elevator.manualSpeed),
        //         () -> m_elevator.stop(),
        //         m_elevator)
        // );

        // elevatorDownButton.whileTrue(
        //     new StartEndCommand(
        //         () -> m_elevator.moveDown(Constants.Elevator.manualSpeed),
        //         () -> m_elevator.stop(),
        //         m_elevator)
        // );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(m_swerve);
    }
}

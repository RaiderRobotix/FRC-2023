package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.drive;
// import frc.robot.commands.pickdown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;
import frc.robot.auton.*;
import frc.robot.auton.Routines.BumpSideSimpleAuto;
import frc.robot.auton.Routines.SimpleAuto;
import frc.robot.auton.Routines.SimpleAutoRamp;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer{
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

    private final JoystickButton hpGrabSequenceButton = new JoystickButton(operator, 2);
    private final JoystickButton toggleGrabberButton = new JoystickButton(operator, 1);
    private final JoystickButton togglePopperButton = new JoystickButton(operator, 4);

     /* Subsystems */
     private final Swerve s_Swerve = new Swerve();
     private final Arm m_arm = new Arm();
     private final Elevator m_elevator = new Elevator();
     private final Pneumatics m_pneumatics = new Pneumatics();
     private final Grabber m_grabber = new Grabber();
     private final AutonSelector m_autoSelector = new AutonSelector();
     private final LimeLight m_limeLight = new LimeLight();
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                driver, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis) * .1, 
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //resetCancoders.onTrue(new InstantCommand(() -> m_swerve.resetModulesToAbsolute()));
        
        /* Operator Buttons */
        hpGrabSequenceButton.and(new Trigger(m_grabber::grabberIsOpen)).onTrue(new HPGrabCone(m_arm, m_elevator, m_grabber));
        toggleGrabberButton.onTrue(new InstantCommand(() -> m_grabber.toggleGrabber()));
        togglePopperButton.onTrue(new InstantCommand(() -> m_pneumatics.togglePopper()));
        
        armOutButton.and(new Trigger(m_arm::upperLimitHit).negate()).whileTrue(
            new StartEndCommand(
                () -> m_arm.extend(Constants.Arm.manualSpeed),
                () -> m_arm.stop(),
                m_arm)
        );

        armInButton.and(new Trigger(m_arm::lowerLimitHit).negate()).whileTrue(
            new StartEndCommand(
                () -> m_arm.retract(Constants.Arm.manualSpeed),
                () -> m_arm.stop(),
                m_arm)
        );

        elevatorUpButton.and(new Trigger(m_elevator::upperLimitHit).negate()).whileTrue(
            new StartEndCommand(
                () -> m_elevator.moveUp(Constants.Elevator.manualSpeed),
                () -> m_elevator.stop(),
                m_elevator)
        );

        elevatorDownButton.and(new Trigger(m_elevator::lowerLimitHit).negate()).whileTrue(
            new StartEndCommand(
                () -> m_elevator.moveDown(Constants.Elevator.manualSpeed),
                () -> m_elevator.stop(),
                m_elevator)
        );

        autoHPButton.onTrue(
                    new ElevatorToHeight(m_elevator, Constants.Elevator.humanPlayerHeight))
            .onTrue(new ArmToPosition(m_arm, Constants.Arm.humanPlayerLength));

        autoLowButton.onTrue(
                    new ElevatorToHeight(m_elevator, Constants.Elevator.lowRowHeight))
            .onTrue(new ArmToPosition(m_arm, Constants.Arm.floorPickupLength));

        autoScoreMidButton.onTrue(
                    new ElevatorToHeight(m_elevator, Constants.Elevator.middleRowHeight))
            .onTrue(new ArmToPosition(m_arm, Constants.Arm.middleRowLength));

        autoScoreHighButton.onTrue(
                    new ElevatorToHeight(m_elevator, Constants.Elevator.topRowHeight))
            .onTrue(new ArmToPosition(m_arm, Constants.Arm.topRowLength));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous

        return m_autoSelector.getCommand(s_Swerve, m_elevator, m_arm, m_pneumatics, m_grabber);
        // return new BumpSideSimpleAuto(s_Swerve, m_pneumatics, m_arm);
        // return new SimpleAutoRamp(s_Swerve, m_pneumatics);
    }
}

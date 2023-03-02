package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.armToLength;
// import frc.robot.commands.drive;
import frc.robot.commands.elevatorToHeight;
import frc.robot.commands.elevatorToHeightNoPID;
// import frc.robot.commands.pickdown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.Pneumatics;
// import frc.robot.subsystems.drivebase.SwerveWheelController;


import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements UniqueConstants{
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    // private final static SwerveWheelController m_controller = new SwerveWheelController();
    private final OperatorInterface m_operatorInterface = new OperatorInterface();
    private final Pneumatics mPneumatics = new Pneumatics(m_operatorInterface);
    // private final Gyro m_gyro = new Gyro();
    private final Elevator m_elevator = new Elevator();
    private final Arm m_arm = new Arm();
    // private final static AutonSelector autonSelector = new AutonSelector();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
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

        // Driver Controls
    // new Trigger(m_operatorInterface::getLeftTrigger)
    //   .whileTrue(new drive(m_controller, m_operatorInterface, slowSpeed));

    // new Trigger(m_operatorInterface::getRightTrigger)
    //   .whileTrue(new drive(m_controller, m_operatorInterface, turboSpeed));

    // new JoystickButton(m_operatorInterface.getXboxController(), XboxController.Button.kY.value)
    // .onTrue(new InstantCommand(() -> Gyro.resetGyro()));
  // new JoystickButton(m_operatorInterface.getXboxController(), XboxController.Button.kA.value)
  //   .and(new Trigger(m_operatorInterface::isPOV))
  //   .whileTrue(new StartEndCommand(
  //     () -> m_controller.drive(ChassisSpeeds.fromFieldRelativeSpeeds(Math.asin(m_operatorInterface.getXboxController().getPOV()), Math.acos(m_operatorInterface.getXboxController().getPOV()), 0, m_controller.getRotation2d())),
  //     () -> m_controller.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_controller.getRotation2d()))));
    
  // new JoystickButton(m_operatorInterface.getXboxController(), XboxController.Button.kB.value)
  //   .and(new Trigger(m_operatorInterface::isPOV)
  //   .whileTrue(new StartEndCommand(
  //     () -> m_controller.setSpeed(Math.cos(m_operatorInterface.getXboxController().getPOV()), Math.sin(m_operatorInterface.getXboxController().getPOV()), 0),
  //     () -> m_controller.stopMotors(),
  //     m_controller)));
      
    // Operator Controls
  new JoystickButton(m_operatorInterface.getOperatorJoystick(), grabberJoystickButton)
    // .and(new Trigger(Arm::isUpperRow).negate())
    .onTrue(new InstantCommand(() -> Pneumatics.toggleGrabberSolenoid()));

  // new JoystickButton(m_operatorInterface.getOperatorJoystick(), grabberJoystickButton)
  // .onTrue(new pickdown());

  new JoystickButton(m_operatorInterface.getOperatorJoystick(), popperJoystickButton)
  .onTrue(new InstantCommand(() -> Pneumatics.togglePopperSolenoid()));

  new JoystickButton(m_operatorInterface.getOperatorJoystick(), armInJoystickButton)
    //.and(new Trigger(Arm::getSensorLow).negate())
    .whileTrue(new StartEndCommand(
      () -> Arm.setMotor(-kArmInSpeed),
      () -> Arm.setMotor(0)));

  new JoystickButton(m_operatorInterface.getOperatorJoystick(), armOutJoystickButton)
    //.and(new Trigger(Arm::getSensorMax).negate())
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
  // .and(new Trigger(Elevator::getSensorLow).negate())
  .whileTrue(new StartEndCommand(
    () -> Elevator.setMotor(kElevatorDownSpeed),
    () -> Elevator.setMotor(0),
    m_elevator));

  new JoystickButton(m_operatorInterface.getOperatorJoystick(), elevatorFloorJoystickButton)
  .whileTrue(new armToLength(kFloorLength))
  .whileTrue(new elevatorToHeight(kFloorHeight));

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
  // new Trigger(m_operatorInterface::getDistanceSensor)
  //   .and(new JoystickButton(m_operatorInterface.getOperatorJoystick(), autoGrabberJoystickButton))
  //   // .debounce(kDistanceSensorDebounceTime)
  //   .onTrue(new pickdown());
  
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
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}

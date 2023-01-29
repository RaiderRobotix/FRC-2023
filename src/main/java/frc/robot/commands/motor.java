// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.OperatorInterface;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class motor extends PIDCommand {
  /** Creates a new motor. */
  OperatorInterface oi;
  public motor(OperatorInterface oi) {
    super(
        // The controller that the command will use
        new PIDController(1, 0.1, 0),
        // This should return the measurement
        () -> Elevator.getSensor(),
        // This should return the setpoint (can also be a constant)
        () -> 1,
        // This uses the output
        output -> {
          Elevator.setMotor(output);
        });
        addRequirements(oi);
        getController().enableContinuousInput(0, 1);
        getController().setTolerance(0.01);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

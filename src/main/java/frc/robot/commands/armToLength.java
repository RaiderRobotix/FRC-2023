// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class armToLength extends PIDCommand implements Constants {
  /** Creates a new armToLength. */
  public armToLength(double length) {
    super(
        // The controller that the command will use
        new PIDController(armKp, armKi, armKd),
        // This should return the measurement
        () -> Arm.getSensor(),
        // This should return the setpoint (can also be a constant)
        () -> length,
        // This uses the output
        output -> {
          if(Arm.getSensorLow() && output < 0){
            output = 0;
          } else if (Arm.getSensorMax() && output > 0){
            output = 0;
          }
          Arm.setMotor(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

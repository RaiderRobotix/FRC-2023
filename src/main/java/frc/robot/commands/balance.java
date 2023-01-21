// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.drivebase.SwerveWheelController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//Command balances robot on charging pad automatically

public class balance extends PIDCommand implements Constants {
  /** Creates a new balance. */
  public balance(SwerveWheelController controller) {
    super(
        // The controller that the command will use
        new PIDController(balanceKp, balanceKi, balanceKd),
        // This should return the measurement
        () -> Gyro.gyro().getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          controller.setSpeed(output, 0, 0);// Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

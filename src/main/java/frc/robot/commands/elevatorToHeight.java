// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class elevatorToHeight extends PIDCommand implements Constants{
  /** Creates a new elevatorToHeight. */
  public elevatorToHeight(double height) {
    super(
        // The controller that the command will use
        new PIDController(elevatorKp, elevatorKi, elevatorKd),
        // This should return the measurement
        () -> Elevator.getSensor(),
        // This should return the setpoint (can also be a constant)
        () -> height,
        // This uses the output
        //Output is the opposite, negative is up and postive is down 
        output -> {
          if(Elevator.getSensorLow() && output < 0){
            output = 0;
          } else if (Elevator.getSensorMax() && output > 0){
            output = 0;
          }
          SmartDashboard.putNumber("output", output);
          Elevator.setMotor(-output);
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
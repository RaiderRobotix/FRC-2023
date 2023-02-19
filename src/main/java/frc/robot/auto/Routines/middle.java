// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.AutonCommands;
import frc.robot.commands.balance;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.drivebase.SwerveWheelController;

/** Add your docs here. */
public class middle extends AutonCommands {
    SwerveWheelController swerveController;
    public middle(String pathName, SwerveWheelController swerveController) {
        super(pathName);
        this.swerveController = swerveController;
        super.setMap("Popper", 
        new InstantCommand(() -> Pneumatics.setPopperSolenoid(true)));
        super.setMap("Balance", new balance(swerveController));
    }
}
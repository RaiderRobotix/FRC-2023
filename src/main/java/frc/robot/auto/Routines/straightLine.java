// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Routines;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutonCommands;
import frc.robot.subsystems.drivebase.SwerveWheelController;

/** Add your docs here. */
public class straightLine extends AutonCommands {
    SwerveWheelController swerveController;
    public straightLine(String pathName, SwerveWheelController swerveController) {
        super(pathName);
        this.swerveController = swerveController;
    }    
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.SwerveWheelController;

/** Add your docs here. */
public class testAuto2 extends autonCommands {
    SwerveWheelController swerveController;
    public testAuto2(String pathName, SwerveWheelController swerveController) {
        super(pathName);
        this.swerveController = swerveController;
        setMap(null, null);
    }    
}

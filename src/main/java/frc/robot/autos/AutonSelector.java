// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.SwerveWheelController;

/** Add your docs here. */
public class AutonSelector {
    public AutonSelector(){
    }

    private static SendableChooser<AutonomousMode> autonomousModeChooser;
    private static Pose2d startingPose;
    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("Test Auto", AutonomousMode.testAuto);

        autoTab.add("autoMode", autonomousModeChooser);
    }

    public Command getCommand(SwerveWheelController swerveController){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case testAuto:
                return new testAuto(swerveController);            
  
            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }

        return null;
    }

    private enum AutonomousMode {
        testAuto  
    }

    public static Pose2d getStartingPose(){
        return startingPose;
    }
}

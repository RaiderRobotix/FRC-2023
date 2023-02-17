// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.auto.Routines.bottom;
import frc.robot.auto.Routines.middle;
import frc.robot.auto.Routines.straightLine;
import frc.robot.auto.Routines.straightLineActions;
import frc.robot.auto.Routines.top;
import frc.robot.subsystems.drivebase.SwerveWheelController;

/** Add your docs here. */
public class AutonSelector implements Constants{
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

    public AutonCommands getCommand(SwerveWheelController swerveController){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case testAuto:
                return new straightLine("Straight Line", swerveController);            
            case straightAutoActions:
                return new straightLineActions("Straight Line with Actions", swerveController);
            case top:
                return new top("Top", swerveController);
            case middle:
                return new middle("Middle", swerveController);
            case bottom:
                return new bottom("Bottom", swerveController);        
            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }

        return null;
    }

    private enum AutonomousMode {
        testAuto,
        straightAutoActions,
        top,
        middle,
        bottom
    }

    public static Pose2d getStartingPose(){
        return startingPose;
    }
}

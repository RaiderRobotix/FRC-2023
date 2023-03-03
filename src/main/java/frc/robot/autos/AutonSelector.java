// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.UniqueConstants;
import frc.robot.autos.Routines.bottom;
import frc.robot.autos.Routines.middle;
import frc.robot.autos.Routines.straightLine;
import frc.robot.autos.Routines.straightLineActions;
import frc.robot.autos.Routines.straightLineLeft;
import frc.robot.autos.Routines.top;

/** Add your docs here. */
public class AutonSelector implements UniqueConstants{
    public AutonSelector(){
    }

    private static SendableChooser<AutonomousMode> autonomousModeChooser;
    private static Pose2d startingPose;
    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("Test Straight Line", AutonomousMode.testAuto);
        autonomousModeChooser.addOption("Test Straight Line Left", AutonomousMode.testAutoLeft);
        autonomousModeChooser.addOption("Test Straight Line w 90", AutonomousMode.testAutoW90);
        autonomousModeChooser.addOption("Test Straight Line w 180", AutonomousMode.testAutoW180);
        autonomousModeChooser.addOption("Test Straight Line with Actions", AutonomousMode.straightAutoActions);
        autonomousModeChooser.addOption("Top", AutonomousMode.top);
        autonomousModeChooser.addOption("Middle", AutonomousMode.middle);
        autonomousModeChooser.addOption("Bottom", AutonomousMode.bottom);

        autoTab.add("autoMode", autonomousModeChooser);
    }

    public AutonCommands getCommand(SwerveWheelController swerveController){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case testAuto:
                return new straightLine("Straight Line", swerveController);
            case testAutoLeft:
                return new straightLineLeft("Straight Line Left", swerveController);
            case testAutoW90:
                return new straightLine("Straight Line w 90", swerveController);
            case testAutoW180:
                return new straightLine("Straight Line w 180", swerveController);            
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
        testAutoLeft,
        testAutoW90,
        testAutoW180,
        straightAutoActions,
        top,
        middle,
        bottom
    }

    public static Pose2d getStartingPose(){
        return startingPose;
    }
}

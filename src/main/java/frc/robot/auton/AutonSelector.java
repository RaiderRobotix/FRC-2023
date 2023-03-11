// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auton.Routines.BumpSideSimpleAuto;
import frc.robot.auton.Routines.SimpleAutoRamp;
import frc.robot.auton.Routines.StraightLineActions;
import frc.robot.auton.Routines.Test180;
import frc.robot.auton.Routines.Test90;
import frc.robot.auton.Routines.PopBalance;
import frc.robot.auton.Routines.PopCrossBalance;
import frc.robot.auton.Routines.PopCrossBridgeGrab;
// import frc.robot.autos.Routines.bottom;
// import frc.robot.autos.Routines.middle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutonSelector {
    private SendableChooser<AutonomousMode> autonomousModeChooser;
    private Pose2d startingPose;
    public AutonSelector(){
        ShuffleboardTab autoTab = Shuffleboard.getTab("default");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("Pop Cross Bridge Balance", AutonomousMode.CrossBridgeBalance);  
        autonomousModeChooser.addOption("Pop Balance", AutonomousMode.Balance);        // autonomousModeChooser.addOption(" Side Straight", AutonomousMode.simpleStraight);
        autonomousModeChooser.addOption("Bump Side Straight", AutonomousMode.simpleStraight);        // autonomousModeChooser.addOption(" Side Straight", AutonomousMode.simpleStraight);
        autonomousModeChooser.addOption("Test Path Planner Straight Line", AutonomousMode.testPathPlanner);        // autonomousModeChooser.addOption(" Side Straight", AutonomousMode.simpleStraight);
        autonomousModeChooser.addOption("Test Path Planner 90 Line", AutonomousMode.testAutoW90);        // autonomousModeChooser.addOption(" Side Straight", AutonomousMode.simpleStraight);
        autonomousModeChooser.addOption("Test Path Planner 180 Line", AutonomousMode.testAutoW180);        // autonomousModeChooser.addOption(" Side Straight", AutonomousMode.simpleStraight);
        //autonomousModeChooser.addOption("Pop Cross Bridge Grab", AutonomousMode.PopCrossBridgeGrab);
        autoTab.add("autoMode", autonomousModeChooser).withSize(5, 2);
    }
    
    
    public Command getCommand(Swerve m_swerve, frc.robot.subsystems.Elevator m_elevator, Arm m_arm, Pneumatics m_pneumatics){
        AutonomousMode mode = autonomousModeChooser.getSelected();
        // System.out.println("passed");


        switch (mode) {
            case simpleStraight:   
                return new BumpSideSimpleAuto(m_swerve, m_pneumatics, m_arm);
            case testPathPlanner:
                return new StraightLineActions(m_swerve, m_pneumatics);
            case testAutoW90:
                return new Test90(m_swerve, m_pneumatics);
            case testAutoW180:
                return new Test180(m_swerve, m_pneumatics); 
            case Balance:
                return new PopBalance(m_swerve, m_pneumatics);   
            case CrossBridgeBalance:
                return new PopCrossBalance(m_swerve, m_pneumatics);   
            case PopCrossBridgeGrab:
                return new PopCrossBridgeGrab(m_swerve ,m_pneumatics);     
            // case straightAutoActions:
            //     return new straightLineActions("Straight Line with Actions", m_sweve);
            // case top:
            //     return new top("Top", m_sweve, m_elevator, m_arm);
            // case middle:
            //     return new middle("Middle", m_sweve);
            // case bottom:
            //     return new bottom("Bottom", m_sweve);        
            default:
                System.out.println("ERROR: unexpected auto mode: " + mode);
                break; 
        }

        return null;
    }

    private enum AutonomousMode {
        middle,
        simpleStraight,
        testPathPlanner,
        testAutoW90,
        testAutoW180,
        Balance,
        CrossBridgeBalance,
        PopCrossBridgeGrab
    }

    // public static Pose2d getStartingPose(){
    //     return startingPose;
    // }
}

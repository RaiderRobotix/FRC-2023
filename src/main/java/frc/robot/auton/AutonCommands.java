package frc.robot.auton;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/** Add your docs here. */
public class AutonCommands{
    PathPlannerTrajectory path;
    HashMap<String, Command> eventMap = new HashMap<>();

    public AutonCommands(String pathName){
        this.path = PathPlanner.loadPath(pathName,
        new PathConstraints(5, 3));
    }

    public void setMap(String name, Command command){
        eventMap.put(name, command);
    }

    public PathPlannerTrajectory getPath(){
        return path;
    }

    public HashMap<String, Command> getEventMap(){
        return eventMap;
    }
}

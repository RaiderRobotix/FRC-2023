// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.UniqueConstants;

/** Add your docs here. */
public class AutonCommands implements UniqueConstants{
    PathPlannerTrajectory path;
    HashMap<String, Command> eventMap = new HashMap<>();

    public AutonCommands(String pathName){
        this.path = PathPlanner.loadPath(pathName,
        new PathConstraints(kPhysicalDriveMaxSpeed, kMaxAccelerationMetersPerSecondSquared));
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

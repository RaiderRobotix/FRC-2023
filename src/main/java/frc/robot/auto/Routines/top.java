// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.AutonCommands;
import frc.robot.commands.armToLength;
import frc.robot.commands.balance;
import frc.robot.commands.elevatorToHeight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.drivebase.SwerveWheelController;

/** Add your docs here. */
public class top extends AutonCommands {
    SwerveWheelController swerveController;
    public top(String pathName, SwerveWheelController swerveController) {
        super(pathName);
        this.swerveController = swerveController;
        //TODO turn popper off
        super.setMap("Popper", 
        new InstantCommand(() -> Pneumatics.setPopperSolenoid(true)));
        super.setMap("Elevator to Floor 1", 
        new elevatorToHeight(kFloorHeight));
        super.setMap("Arm to Floor 1", 
        new armToLength(kFloorLength));
        super.setMap("Open Grabber 1", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(true)));

        super.setMap("Close Grabber 1", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(false)));
        super.setMap("Elevator to Game Piece 1", 
        new elevatorToHeight(kUpperRowHeight));
        super.setMap("Arm to Game Piece 1", 
        new armToLength(kUpperRowLength));

        super.setMap("Open Grabber 2", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(true)));

        super.setMap("Elevator to Floor 2", 
        new elevatorToHeight(kFloorLength));
        super.setMap("Arm to Floor 2", 
        new armToLength(kFloorLength));

        super.setMap("Close Grabber 2", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(false)));
        super.setMap("Elevator to Game Piece 2", 
        new elevatorToHeight(kUpperRowHeight));
        super.setMap("Arm to Game Piece 2", 
        new armToLength(kUpperRowLength));

        super.setMap("Open Grabber 3", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(true)));

        super.setMap("Balance", new balance(swerveController));

    }
}

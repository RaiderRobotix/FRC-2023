// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.Routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.UniqueConstants;
import frc.robot.autos.AutonCommands;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.balance;
import frc.robot.commands.ElevatorToHeight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class top extends AutonCommands implements UniqueConstants {
    Swerve swerveController;
    public top(String pathName, Swerve swerveController, Elevator m_elevator, Arm m_arm) {
        super(pathName);
        this.swerveController = swerveController;
        //TODO turn popper off
        super.setMap("Popper", 
        new InstantCommand(() -> Pneumatics.setPopperSolenoid(true)));
        super.setMap("Elevator to Floor 1", 
        new ElevatorToHeight(m_elevator, kFloorHeight));
        super.setMap("Arm to Floor 1", 
        new ArmToPosition(m_arm, kFloorLength));
        super.setMap("Open Grabber 1", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(true)));

        super.setMap("Close Grabber 1", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(false)));
        super.setMap("Elevator to Game Piece 1", 
        new ElevatorToHeight(m_elevator, kUpperRowHeight));
        super.setMap("Arm to Game Piece 1", 
        new ArmToPosition(m_arm, kUpperRowLength));

        super.setMap("Open Grabber 2", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(true)));

        super.setMap("Elevator to Floor 2", 
        new ElevatorToHeight(m_elevator, kFloorLength));
        super.setMap("Arm to Floor 2", 
        new ArmToPosition(m_arm, kFloorLength));

        super.setMap("Close Grabber 2", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(false)));
        super.setMap("Elevator to Game Piece 2", 
        new ElevatorToHeight(m_elevator, kUpperRowHeight));
        super.setMap("Arm to Game Piece 2", 
        new ArmToPosition(m_arm, kUpperRowLength));

        super.setMap("Open Grabber 3", 
        new InstantCommand(() -> Pneumatics.setGrabberSolenoid(true)));

        super.setMap("Balance", new balance(swerveController));

    }
}

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.commands.*;
// import frc.robot.subsystems.*;
// import frc.robot.Constants;


// public class ToPreset extends CommandBase {
//   Elevator m_elevator;
//   Arm m_arm;

//   String preset;
//   boolean isDone;
//   /** Creates a new ToPreset. */
//   public ToPreset(Elevator elevator, Arm arm, String preset ) {
//     m_elevator = elevator;
//     m_arm = arm;
//     this.preset = preset;

//     addRequirements(m_elevator);
//     addRequirements(m_arm);
//     // Use addRequirements() here to declare subsystem dependencies.
  
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     isDone = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(preset.equals("lowRow")){
//       new ElevatorToHeight(m_elevator, Constants.Elevator.lowRowHeight);
//       new ArmToPosition(m_arm, Constants.Arm.floorPickupLength);
//     } else if (preset.equals("middleRow")){
//       new ElevatorToHeight(m_elevator, Constants.Elevator.middleRowHeight);
//       new ArmToPosition(m_arm, Constants.Arm.middleRowLength);
//     } else if (preset.equals("topRow")){
//       new ElevatorToHeight(m_elevator, Constants.Elevator.topRowHeight);
//       new ArmToPosition(m_arm, Constants.Arm.topRowLength);
//     } else if (preset.equals("humanPlayer")){
//       new ElevatorToHeight(m_elevator, Constants.Elevator.humanPlayerHeight);
//       new ArmToPosition(m_arm, Constants.Arm.humanPlayerLength);
//     }
//     isDone = true;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return isDone;
//   }
// }

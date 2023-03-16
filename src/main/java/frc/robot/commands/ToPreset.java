// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ToPreset extends SequentialCommandGroup {
//   /** Creates a new ToPreset. */
//   public ToPreset(Elevator elevator, Arm arm, String preset ) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     if(preset.equals("lowRow")){
//       double height = Constants.Elevator.lowRowHeight;
//       double length = Constants.Arm.floorPickupLength;
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

//     addCommands(
//     );
//   }
// }

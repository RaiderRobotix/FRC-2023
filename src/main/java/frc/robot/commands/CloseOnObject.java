// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants;
// import frc.robot.subsystems.Grabber;

// public class CloseOnObject extends CommandBase {
//   /** Creates a new CloseOnObject. */

//   private Timer timer;
//   private double seconds;
//   private boolean isDone;
//   private Grabber m_grabber;

//   public CloseOnObject(Grabber m_grabber, Timer seconds) {
    
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.seconds = seconds;
//     this.m_grabber = m_grabber;
//     this.timer = new Timer();
//     addRequirments(m_grabber);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     isDone = false;
//     timer.reset();
//     timer.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(timer.get() < seconds){
//       m_grabber.closeIfObjectDetected();
//     }
//     else{
//       timer.stop();
//       isDone = true;
//     }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     timer.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return isDone;
//   }
// }

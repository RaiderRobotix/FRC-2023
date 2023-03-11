// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.Routines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.ArmToPosition;
import frc.robot.commands.ElevatorToHeight;



public class DriveBackRaiseArm extends CommandBase {
  /** Creates a new DriveAtSpeed. */
  private Translation2d speeds;
  private double seconds;
  private Timer timer;
  private boolean isDone;
  private Swerve m_swerve;
  private Grabber m_grabber;
  private Arm m_arm;
  private Elevator m_elevator;
  public DriveBackRaiseArm(Swerve m_swerve, Arm m_arm, Elevator m_elevator, Grabber m_grabber,double speed, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speeds = new Translation2d(-speed * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
    this.seconds = seconds;
    this.m_swerve = m_swerve;
    this.m_grabber = m_grabber;
    this.m_arm = m_arm;
    this.m_elevator = m_elevator;
    this.timer = new Timer();
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    timer.reset();
    timer.start();
    if(m_grabber.grabberIsClosed()){
      CommandScheduler.getInstance().schedule(new ArmToPosition(m_arm, Constants.Arm.humanPlayerLength));
      CommandScheduler.getInstance().schedule(new ElevatorToHeight(m_elevator, Constants.Elevator.humanPlayerHeight));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_grabber.grabberIsClosed() && timer.get() < this.seconds) {
      m_swerve.drive(speeds, 0, false, true);
    } else {
      timer.stop();
      m_swerve.stop();
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}

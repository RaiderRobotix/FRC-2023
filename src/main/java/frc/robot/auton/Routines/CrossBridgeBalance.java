// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.Routines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;

public class CrossBridgeBalance extends CommandBase {
    /** Creates a new DriveAtSpeed. */
    private Translation2d speeds;
    private double seconds;
    private Timer timer;
    private boolean isDone;
    private Swerve m_swerve;
    private int step;
    public CrossBridgeBalance(Swerve m_swerve) {
        // Use addRequirements() here to declare subsystem dependencies.
        // this.speeds = new Translation2d(xSpeed * Constants.SwerveConstants.maxSpeed, ySpeed * Constants.SwerveConstants.maxSpeed);
        this.m_swerve = m_swerve;
        this.timer = new Timer();
        this.step = 0;
        addRequirements(m_swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isDone = false;
        step = 0;
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
      if(step == 0) {
        // drive forward until on ramp, angled up
        this.speeds = new Translation2d(0.4 * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
        m_swerve.drive(speeds, 0, false, true);
        if(m_swerve.getPitch() <= -12){
          m_swerve.stop();
          step++;
        }
      }
      else if(step == 1){
        // drive forward until angled down on ramp
        this.speeds = new Translation2d(0.2 * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
        m_swerve.drive(speeds, 0, false, true);
        if(m_swerve.getPitch() >= -2){
          m_swerve.stop();
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        }
      } 
      else if(step == 2){
        if(timer.get() >= 0.5){
          m_swerve.stop();
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        }
      }
      else if(step == 3){
        // drive forward until robot is flat, past bridge
        this.speeds = new Translation2d(0.2 * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
        m_swerve.drive(speeds, 0, false, true);
        if(m_swerve.getPitch() < 2){
          m_swerve.stop();
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        }
      } 
      else if(step == 4){
        // drive forward past mobility line
        this.speeds = new Translation2d(0.2 * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
        m_swerve.drive(speeds, 0, false, true);
        if(timer.get() >= 0.7){
          m_swerve.stop();
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        }
      } 
      // same as SimpleBalance routine from here, reverse direction
      else if(step == 5) {
        this.speeds = new Translation2d(-0.4 * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
        m_swerve.drive(speeds, 0, false, true);
        if(m_swerve.getPitch() >= 12){
          m_swerve.stop();
          step++;
        }
      }
      else if(step == 6){
        this.speeds = new Translation2d(-0.2 * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
        m_swerve.drive(speeds, 0, false, true);
        if(m_swerve.getPitch() <= 5){
          m_swerve.stop();
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        }
      } 
      else if(step == 7){
        if(timer.get() >= 0.5){
          m_swerve.stop();
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        
        }
      }
      else if(step == 8){
        this.speeds = new Translation2d(0.20 * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
        m_swerve.drive(speeds, 0, false, true);
        if(timer.get() >= 0.75){
          m_swerve.stop();
          m_swerve.setAngle(90);
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        }
      }
      else if (step == 9) {
        if (timer.get() > 0.5) {
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        }
      }
      else if (step == 10) {
        // continually adjust
        if (m_swerve.getPitch() <= -2.5) 
        {
          this.speeds = new Translation2d(0.2 * Constants.SwerveConstants.maxSpeed, 0 * Constants.SwerveConstants.maxSpeed);
          m_swerve.drive(speeds, 0, false, true);
          Timer.delay(0.3);
          m_swerve.stop();
          m_swerve.setAngle(90);
          Timer.delay(1.0);
        }
        else 
        {
          m_swerve.stop();
          m_swerve.setAngle(90);
          timer.stop();
          timer.reset();
          timer.start();
          step++;
        }
      }
      else {
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

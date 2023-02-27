package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class armToLengthNoPID extends CommandBase implements Constants {
    
    private final Arm m_arm;

    private double targetLength;
    private double initialPosition;
    private boolean isDone = false;

    public armToLengthNoPID(Arm arm, double position)
    {
        this.m_arm = arm;
        this.targetLength = position;

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        this.isDone = false;
        this.initialPosition = Arm.getSensor();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        // if (targetLength > Constants.Arm.upperSafety || targetLength < Constants.Arm.lowerSafety) 
        // {
        //     end(true);
        // }
  
        if (initialPosition < targetLength)
        {
            if (Arm.getSensor() < targetLength) 
            {
                Arm.setMotor(kArmOutSpeed);
            } 
            else 
            {
                Arm.setMotor(0);
                isDone = true;
            }
        } 
        else if (initialPosition > targetLength) 
        {
            if (Arm.getSensor() > targetLength) 
            {
                Arm.setMotor(kArmInSpeed);
            } 
            else 
            {
                Arm.setMotor(0);
                isDone = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Arm.setMotor(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
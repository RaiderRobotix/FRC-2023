package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmToPosition extends CommandBase {
    
    private final Arm m_arm;

    private double targetLength;
    private double initialPosition;
    private boolean isDone = false;

    public ArmToPosition(Arm arm, double position)
    {
        this.m_arm = arm;
        this.targetLength = position;

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        this.initialPosition = m_arm.getPotValue();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (targetLength > Constants.Arm.upperSafety || targetLength < Constants.Arm.lowerSafety) 
        {
            end(true);
        }
  
        if(initialPosition < targetLength)
        {
            if(m_arm.getPotValue() < targetLength) 
            {
                m_arm.extend(Constants.Arm.autoSpeed);
            } 
            else 
            {
                m_arm.stop();
                isDone = true;
            }
        } 
        else if (initialPosition > targetLength) 
        {
            if(m_arm.getPotValue() > targetLength) 
            {
                m_arm.retract(Constants.Arm.autoSpeed);
            } 
            else 
            {
                m_arm.stop();
                isDone = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_arm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorToHeight extends CommandBase{
    private final Elevator m_elevator;

    private double targetHeight;
    private double initialHeight;
    private boolean isDone = false;

    public ElevatorToHeight(Elevator elevator, double height)
    {
        this.m_elevator = elevator;
        this.targetHeight = height;

        addRequirements(m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        this.initialHeight = m_elevator.getPotValue();
        this.isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (targetHeight > Constants.Elevator.upperSafety || targetHeight < Constants.Elevator.lowerSafety) 
        {
            end(true);
        }
  
        if(initialHeight < targetHeight)
        {
            if(m_elevator.getPotValue() < targetHeight) 
            {
                m_elevator.moveUp(Constants.Elevator.manualSpeed);
            } 
            else 
            {
                m_elevator.stop();
                isDone = true;
            }
        } 
        else if (initialHeight > targetHeight) 
        {
            if(m_elevator.getPotValue() > targetHeight) 
            {
                m_elevator.moveDown(Constants.Elevator.autoSpeed);
            } 
            else 
            {
                m_elevator.stop();
                isDone = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}

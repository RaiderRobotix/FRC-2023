package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class elevatorToHeightNoPID extends CommandBase implements Constants{
    private final Elevator m_elevator;

    private double targetHeight;
    private double initialHeight;
    private boolean isDone = false;

    public elevatorToHeightNoPID(Elevator elevator, double height)
    {
        this.m_elevator = elevator;
        this.targetHeight = height;

        addRequirements(m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        this.initialHeight = Elevator.getSensor();
        this.isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (targetHeight > kElevatorMaxHeight || targetHeight < kElevatorMinHeight) 
        {
            end(true);
        }
  
        if(initialHeight < targetHeight)
        {
            if(Elevator.getSensor() < targetHeight) 
            {
                Elevator.setMotor(kElevatorUpSpeed);
            } 
            else 
            {
                Elevator.setMotor(0);
                isDone = true;
            }
        } 
        else if (initialHeight > targetHeight) 
        {
            if(Elevator.getSensor() > targetHeight) 
            {
                Elevator.setMotor(kElevatorDownSpeed);
            } 
            else 
            {
                Elevator.setMotor(0);
                isDone = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Elevator.setMotor(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
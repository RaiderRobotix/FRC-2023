package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;


public class HPGrabCone extends CommandBase {
    

    private final Elevator m_elevator;
    private final Arm m_arm;

    private boolean isDone;


    public HPGrabCone(Arm arm, Elevator elevator)
    {
        m_arm = arm;
        m_elevator = elevator;

        addRequirements(m_arm);
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() 
    {
        this.isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (Pneumatics.getHpTouchSensor())
        {
            Pneumatics.setGrabberSolenoid(false);
            m_elevator.stop();
            isDone = true;
        }
        else {
            m_elevator.moveDown(0.3);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_arm.stop();
        m_elevator.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
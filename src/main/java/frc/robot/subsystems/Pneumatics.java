package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
    
    private Compressor compressor;

    private Solenoid popperSolenoidOn;
    private Solenoid popperSolenoidOff;

    private boolean popperIsPopped = false;

    public Pneumatics() {
        compressor = new Compressor(Constants.Pneumatics.compressorModule, PneumaticsModuleType.REVPH);
        compressor.enableDigital();

        popperSolenoidOn = new Solenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.popperOnChannel);
        popperSolenoidOff = new Solenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.popperOffChannel);
    }

    public double getPressure()
    {
        return compressor.getPressure();
    }

    public boolean isCompressorEnabled()
    {
        return compressor.isEnabled();
    }

    public void popPopper()
    {
        // TODO: Check values
        popperSolenoidOff.set(false);
        popperSolenoidOn.set(true);

        popperIsPopped = true;
    }

    public void retractPopper()
    {
        // TODO: Check values
        popperSolenoidOff.set(true);
        popperSolenoidOn.set(false);

        popperIsPopped = false;
    }

    public void togglePopper() 
    {
        if (popperIsPopped) {
            retractPopper();
        } else {
            popPopper();
        }
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Pressure:", getPressure());
        SmartDashboard.putString("Popper", popperIsPopped ? "Popped" : "Ready");
    }
}

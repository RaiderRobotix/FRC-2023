package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
    
    private Compressor compressor;

    private Solenoid popperSolenoidOn;
    private Solenoid popperSolenoidOff;

    private boolean popperIsPopped;

    private ShuffleboardTab tab = Shuffleboard.getTab("default");
    private GenericEntry pressureEntry = tab.add("Pressure", 0).withSize(2, 1).getEntry();
    private GenericEntry popperEntry = tab.add("Popper", "").withSize(2, 1).getEntry();


    public Pneumatics() {
        compressor = new Compressor(Constants.Pneumatics.compressorModule, PneumaticsModuleType.REVPH);
        compressor.enableDigital();

        popperSolenoidOn = new Solenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.popperOnChannel);
        popperSolenoidOff = new Solenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.popperOffChannel);
        retractPopper();
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
        popperSolenoidOff.set(true);
        popperSolenoidOn.set(false);

        popperIsPopped = true;
    }

    public void retractPopper()
    {
        popperSolenoidOff.set(false);
        popperSolenoidOn.set(true);

        popperIsPopped = false;
    }

    public void togglePopper() 
    {
        if (!popperIsPopped) {
            popPopper();
        } else {
            retractPopper();
        }
    }

    @Override
    public void periodic()
    {
        pressureEntry.setDouble(getPressure());
        popperEntry.setString(popperIsPopped ? "Popped" : "Ready");
    }
}
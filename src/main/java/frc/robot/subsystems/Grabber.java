package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    
    private Solenoid grabberSolenoidOn;
    private Solenoid grabberSolenoidOff;

    private boolean grabberIsOpen = true;

    private DigitalInput distanceSensor;
    private DigitalInput hpTouchSensor;

    public Grabber()
    {
        grabberSolenoidOn = new Solenoid(PneumaticsModuleType.REVPH, Constants.Grabber.grabberOnChannel);
        grabberSolenoidOff = new Solenoid(PneumaticsModuleType.REVPH, Constants.Grabber.grabberOffChannel);
    
        distanceSensor = new DigitalInput(Constants.Grabber.distanceSensorChannel);
        hpTouchSensor = new DigitalInput(Constants.Grabber.hpTouchSensorChannel);
    }

    public void openGrabber()
    {
        // TODO: Check values.
        grabberSolenoidOff.set(true);
        grabberSolenoidOn.set(false);

        grabberIsOpen = true;
    }

    public void closeGrabber()
    {
        // TODO: Check values
        grabberSolenoidOff.set(false);
        grabberSolenoidOn.set(true);

        grabberIsOpen = false;
    }

    public void toggleGrabber()
    {
        if (grabberIsOpen)  {
            closeGrabber();
        } else {
            openGrabber();
        }
    }

    public boolean grabberIsOpen() {
        return grabberIsOpen;
    }

    public boolean getDistanceSensor() 
    {
        return !distanceSensor.get();
    }

    public boolean getHpTouchSensor() 
    {
        return hpTouchSensor.get();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putString("Grabber", grabberIsOpen ? "Open" : "Closed");
        SmartDashboard.putBoolean("Infrared", getDistanceSensor());
        SmartDashboard.putBoolean("HP Touch Sensor", getHpTouchSensor());
    }
}

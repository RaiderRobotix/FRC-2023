package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private WPI_TalonFX motor;
    private AnalogPotentiometer pot;

    public Elevator()
    {
        this.motor = new WPI_TalonFX(Constants.Elevator.talonDeviceNumber);
        this.pot = new AnalogPotentiometer(Constants.Elevator.potentiometerChannel);
    }

    public double getPotValue()
    {
        return this.pot.get();
    }

    public void moveUp(double speed)
    {
        this.setMotor(-speed);
    }

    public void moveDown(double speed)
    {
        this.setMotor(speed);
    }

    public void stop()
    {
        this.setMotor(0);
    }

    /**
     * Negative speed = going up
     * Positive speed = going down
     * @param speed
     */
    private void setMotor(double speed)
    {
        if (lowerLimitHit() && speed > 0)
        {
            motor.set(0);
        } 
        else if (upperLimitHit() && speed < 0) 
        {
            motor.set(0);
        } 
        else 
        {
            motor.set(speed);
        }
    }

    public boolean upperLimitHit()
    {
        return getPotValue() >= Constants.Elevator.upperSafety;
    }

    public boolean lowerLimitHit()
    {
        return getPotValue() <= Constants.Elevator.lowerSafety;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator Pot", getPotValue());
        SmartDashboard.putBoolean("Elevator Safety Hit", upperLimitHit() || lowerLimitHit());
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private WPI_TalonFX motor;
    private AnalogPotentiometer pot;

    public Arm()
    {
        this.motor = new WPI_TalonFX(Constants.Arm.talonDeviceNumber);
        this.pot = new AnalogPotentiometer(Constants.Arm.potentiometerChannel);
    }

    public double getPotValue()
    {
        return this.pot.get();
    }

    public void extend(double speed)
    {
        this.setMotor(speed);
    }

    public void retract(double speed)
    {
        this.setMotor(-speed);
    }

    public void stop()
    {
        this.setMotor(0);
    }

    /**
     * Positive speed = extending
     * Negative speed = retracting
     * @param speed
     */
    private void setMotor(double speed)
    {
        // if (lowerLimitHit() && speed < 0)
        // {
        //     motor.set(0);
        // } 
        // else if (upperLimitHit() && speed > 0) 
        // {
        //     motor.set(0);
        // } 
        // else 
        // {
            motor.set(speed);
        // }
    }

    public boolean upperLimitHit()
    {
        return getPotValue() >= Constants.Arm.upperSafety;
    }

    public boolean lowerLimitHit()
    {
        return getPotValue() <= Constants.Arm.lowerSafety;
    }
    
    public boolean armInScoringPosition()
    {
        return getPotValue() >= Constants.Arm.middleRowLength;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Arm Pot", getPotValue());
        SmartDashboard.putBoolean("Arm Safety Hit", upperLimitHit() || lowerLimitHit());
    }
}
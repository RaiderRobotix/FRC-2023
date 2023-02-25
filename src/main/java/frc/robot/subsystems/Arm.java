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

    /**
     * Positive speed = extending
     * Negative speed = retracting
     * @param speed
     */
    public void setMotor(double speed)
    {
        if (getPotValue() <= Constants.Arm.lowerSafety && speed < 0)
        {
            motor.set(0);
        } 
        else if (getPotValue() >= Constants.Arm.upperSafety && speed > 0) 
        {
            motor.set(0);
        } 
        else 
        {
            motor.set(speed);
        };
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Arm Pot", getPotValue());
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Pneumatics extends SubsystemBase implements Constants {
  /** Creates a new ExampleSubsystem. */

  private static Compressor pneumatics;
  private Solenoid solenoid;
  private static Solenoid grabberSolenoidOn;
  private static Solenoid grabberSolenoidOff;

  private static Solenoid popperSolenoidOn;
  private static Solenoid popperSolenoidOff;

  // private AnalogInput sonic;

  // private DigitalInput sensor;

  private OperatorInterface oi;


  public Pneumatics(int channel) {
    pneumatics = new Compressor(1, PneumaticsModuleType.REVPH);
    pneumatics.enableHybrid(minPSI, maxPSI);
    //this.pneumatics.enableDigital();
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, channel); 
  }

  public Pneumatics(OperatorInterface oi) {
    pneumatics = new Compressor(1, PneumaticsModuleType.REVPH);
    // pneumatics.enableHybrid(minPSI, maxPSI);
    // pneumatics.enableAnalog(minPSI, maxPSI);
    this.pneumatics.enableDigital();
    // pneumatics.disable();

    grabberSolenoidOn = new Solenoid(PneumaticsModuleType.REVPH, grabberSolenoidOnChannel); 
    grabberSolenoidOff = new Solenoid(PneumaticsModuleType.REVPH, grabberSolenoidOffChannel);

    popperSolenoidOn = new Solenoid(PneumaticsModuleType.REVPH, popperSolenoidOnChannel);
    popperSolenoidOn = new Solenoid(PneumaticsModuleType.REVPH, popperSolenoidOffChannel);

    // sonic = new AnalogInput(0);
    // sensor = new DigitalInput(0);
    this.oi = oi;
  }

  public double getPneumaticsPressure(){
    return pneumatics.getPressure();
  }

  public boolean isPneumaticsEnabled(){
    return pneumatics.isEnabled();
  }

  public static boolean getGrabberSolenoid(){
    return grabberSolenoidOn.get();
  }

  public static void toggleGrabberSolenoid(){
    System.out.println("Passed");
    grabberSolenoidOff.set(getGrabberSolenoid());
    grabberSolenoidOn.set(!getGrabberSolenoid());
    // togglePopperSolenoid(); 
  }

  public static void setGrabberSolenoid(boolean state){
    grabberSolenoidOn.set(state);
    grabberSolenoidOff.set(!state);
  }

  public static boolean getPopperSolenoid(){
    return popperSolenoidOn.get();
  }

  public static void togglePopperSolenoid(){
    popperSolenoidOff.set(getPopperSolenoid());
    popperSolenoidOn.set(!getPopperSolenoid());
  }



  @Override
  public void periodic() {
    // setSolenoidValue(true);

    SmartDashboard.putNumber(this.getName() + "Pneumatics Pressure", getPneumaticsPressure());
    SmartDashboard.putBoolean("Grabber Solenoid", getGrabberSolenoid());
    SmartDashboard.putBoolean("Popper Solenoid", getPopperSolenoid());

    // SmartDashboard.putNumber("sonic", sonic.getValue());
    // if (!sensor.get() && !oi.getOperatorJoystick().getRawButton(5)) {
    //   Pneumatics.setGrabberSolenoid(true);
    // } else if (oi.getOperatorJoystick().getRawButton(3)) {
    //   Pneumatics.setGrabberSolenoid(false);
    // }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
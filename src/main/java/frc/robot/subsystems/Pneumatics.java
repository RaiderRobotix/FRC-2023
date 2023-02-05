// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;




public class Pneumatics extends SubsystemBase implements Constants {
  /** Creates a new ExampleSubsystem. */

  private static Compressor pneumatics;
  private Solenoid solenoid;
// private static Solenoid grabberSolenoidOn;
//   private static Solenoid grabberSolenoidOff;

//   private static Solenoid popperSolenoidOn;
//   private static Solenoid popperSolenoidOff;

  private static DoubleSolenoid grabberSolenoid;
  private static DoubleSolenoid popperSolenoid;

  private static Value value;

  // private AnalogInput sonic;

  // private DigitalInput sensor;

  private OperatorInterface oi;


  public Pneumatics(int enterChannel, int closeChannel) {
    pneumatics = new Compressor(1, PneumaticsModuleType.REVPH);
    pneumatics.enableDigital();


    
  }

  public Pneumatics(OperatorInterface oi) {
    pneumatics = new Compressor(1, PneumaticsModuleType.REVPH);
    
    this.pneumatics.enableDigital();
    
    grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, grabberSolenoidOnChannel, grabberSolenoidOffChannel);
    popperSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, popperSolenoidOnChannel, popperSolenoidOffChannel);

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

  public static String getGrabberSolenoid(){
    return "" + grabberSolenoid.get();
  }

  public static void toggleGrabberSolenoid(){
    System.out.println("Passed");
    grabberSolenoid.toggle();
  }

  public static void setGrabberSolenoid(boolean state){
    if(state){
        grabberSolenoid.set(Value.kForward);}
    else{
        grabberSolenoid.set(Value.kReverse);
    }
    
  }

  public static String getPopperSolenoid(){
    return "" + popperSolenoid.get();
  }

  public static void togglePopperSolenoid(){
    popperSolenoid.toggle();
  }



  @Override
  public void periodic() {

    SmartDashboard.putNumber(this.getName() + "Pneumatics Pressure", getPneumaticsPressure());
    SmartDashboard.putString("Grabber Solenoid", getGrabberSolenoid());
    SmartDashboard.putString("Popper Solenoid", getPopperSolenoid());

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
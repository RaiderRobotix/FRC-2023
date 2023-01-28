// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Pneumatics extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private Compressor pneumatics;
  private Solenoid solenoid;


  public Pneumatics(int module, PneumaticsModuleType moduleType) {
    this.pneumatics = new Compressor(module, moduleType);
    //this.pneumatics.enableHybrid(0.0,80.0);
    this.pneumatics.enableDigital();
    this.solenoid = new Solenoid(moduleType, 2);


  }

  public double getPneumaticsPressure(){
    return pneumatics.getPressure();
  }

  public boolean isPneumaticsEnabled(){
    return pneumatics.isEnabled();
  }

  public boolean getSolenoidValue(){
    return solenoid.get();
  }

  public void setSolenoidValue(boolean on){
    solenoid.set(on);
    System.out.println("passed");
  }




  @Override
  public void periodic() {
    // setSolenoidValue(true);
    SmartDashboard.putNumber(this.getName() + "Pneumatics Pressure", getPneumaticsPressure());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
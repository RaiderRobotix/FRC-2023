// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  /** Creates a new limeLight. */

  public double tv; 
  public double tx;
  public double ty;
  public double ta;
  public double tid;
  public Number[] camtrain;
  private String name =  "limelight";
  private ShuffleboardTab limeLight;

  public LimeLight() {
    ShuffleboardTab limeLight = Shuffleboard.getTab("limeLight");
    updateValues();
  }

  public void updateValues(){
    this.tv = NetworkTableInstance.getDefault().getTable(name).getEntry("tv").getDouble(0); // If Target is found
    this.tx = NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0); // Horazontal Offset
    this.ty = NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0); // Vertical Offset
    this.ta = NetworkTableInstance.getDefault().getTable(name).getEntry("ta").getDouble(0); // Target Area
    this.tid = NetworkTableInstance.getDefault().getTable(name).getEntry("tid").getDouble(0); //Target ID
    this.camtrain = NetworkTableInstance.getDefault().getTable(name).getEntry("camtrain").getNumberArray(null);
  }

  public double getTv(){
    return tv;
  } 
  public double getTx(){
    return tx;
  } 
  public double getTy(){
    return ty;
  } 
  public double getTa(){
    return ta;
  } 
  public double getTID(){
    return tid;
  } 



  @Override
  public void periodic() {
    Shuffleboard.selectTab("limeLight");
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putNumber("tid", tid);
    System.out.println(camtrain);
    updateValues();
    // This method will be called once per scheduler run
  }
}
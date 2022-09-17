// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveBase;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;



public class DriveTrain extends SubsystemBase {

  private DifferentialDrive driveSystem = driveConstants.driveSystem;
  
  private static final TalonFX[] TalonFxMotor = {driveConstants.LEFT_FRONT_TALONFX, driveConstants.LEFT_BACK_TALONFX,
    driveConstants.RIGHT_FRONT_TALONFX, driveConstants.RIGHT_BACK_TALONFX};

  private static final SupplyCurrentLimitConfiguration currentLim = new SupplyCurrentLimitConfiguration(true,40,
  40,0);

  public DriveTrain() {

      driveSystem.setDeadband(0.15);

      for(TalonFX motor : TalonFxMotor){
        motor.configFactoryDefault();
        motor.clearStickyFaults();
        motor.configSupplyCurrentLimit(currentLim);
        
      }
  }
  

  //Motor Temprature
  public double getTemp(int motorId){
    switch(motorId){
      case 1: return driveConstants.LEFT_FRONT_TALONFX.getTemperature();
      case 2: return driveConstants.LEFT_BACK_TALONFX.getTemperature();
      case 3: return driveConstants.RIGHT_FRONT_TALONFX.getTemperature();
      case 4: return driveConstants.RIGHT_BACK_TALONFX.getTemperature();
    }
    return 0.0;
  }

  //Used to set the same speed for both sides
  public void setSpeed(double speed){
    
    driveSystem.tankDrive(speed, -speed);

  }
  //Used to Set Speed different for each side
  public void setSpeed(double leftSpeed, double rightSpeed){
   
    driveSystem.tankDrive(leftSpeed, -rightSpeed);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

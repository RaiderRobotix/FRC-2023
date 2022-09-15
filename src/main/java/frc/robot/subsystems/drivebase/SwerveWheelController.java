// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveWheelController extends SubsystemBase implements drivebaseConstants {
  private static SwerveWheelController instance;

  /** Creates a new drivebase. */
  public SwerveWheelController() {
    // Location of modules relative to the centre of the robot
    Translation2d frontLeftLocation = new Translation2d(frontLeftLocationX, frontLeftLocationY);
    Translation2d frontRightLocation = new Translation2d(frontRightLocationX, frontRightLocationY);
    Translation2d backLeftLocation = new Translation2d(backLeftLocationX, backLeftLocationY);
    Translation2d backRightLocation = new Translation2d(backRightLocationX, backRightLocationY);

    System.out.println(frontLeftLocation.getNorm());

    // Creates kinematics object using the above module location
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    System.out.println(kinematics);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static SwerveWheelController getInstance() {
    if (instance == null) {
      instance = new SwerveWheelController();
      // instance.setDefaultCommand(new TeleopDrive());
    }

    return instance;
  }
}

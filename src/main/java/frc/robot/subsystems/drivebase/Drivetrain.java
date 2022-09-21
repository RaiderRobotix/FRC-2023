// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Gyro;

public class Drivetrain extends SubsystemBase implements drivebaseConstants {
  /** Creates a new Drivetrain. */
  private final static Translation2d frontLeftLocation = new Translation2d(frontLeftLocationX, frontLeftLocationY);
  private final static Translation2d frontRightLocation = new Translation2d(frontRightLocationX, frontRightLocationY);
  private final static Translation2d backLeftLocation = new Translation2d(backLeftLocationX, backLeftLocationY);
  private final static Translation2d backRightLocation = new Translation2d(backRightLocationX, backRightLocationY);

  private final static SwerveWheel frontLeftModule = new SwerveWheel(1, 2, 3, "leftFront");
  private final static SwerveWheel frontRightModule = new SwerveWheel(4, 5, 6, "rightFront");
  private final static SwerveWheel backLeftModule = new SwerveWheel(7, 8, 9, "leftBack");
  private final static SwerveWheel backRightModule = new SwerveWheel(10, 11, 12, "rightBack");

  private final static SwerveDriveKinematics kinemantics = new SwerveDriveKinematics(frontLeftLocation,
      frontRightLocation,
      backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinemantics, Gyro.gyro().getRotation2d());
  private final SimDeviceSim gyroSim = new SimDeviceSim("Angle", 0);;

  public Drivetrain() {
    Gyro.gyro().reset();
  }

  public static void drive(double x, double y, double radians) {
    var SwerveModuleState = kinemantics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, radians, Gyro.gyro().getRotation2d()));

    frontLeftModule.set(SwerveModuleState[0]);
    frontRightModule.set(SwerveModuleState[1]);
    backLeftModule.set(SwerveModuleState[2]);
    backRightModule.set(SwerveModuleState[3]);

  }

  public void update() {
    odometry.update(Gyro.gyro().getRotation2d(), frontLeftModule.getState(), frontRightModule.getState(),
        backLeftModule.getState(), backRightModule.getState());
  }

  @Override
  public void simulationPeriodic() {
    // frontLeftModule.simulationPeriodic(0.02);
    // frontLeftModule.simulationPeriodic(0.02);
    // frontLeftModule.simulationPeriodic(0.02);
    // frontLeftModule.simulationPeriodic(0.02);

    SwerveModuleState [] wheelStates = {
      frontLeftModule.getState();
      frontRightModule.getState();
      backLeftModule.getState();
      backRightModule.getState();
    }

    var chassisSpeed = kinemantics.toChassisSpeeds(wheelStates);
    double chassisRotateSpeed = chassisSpeed.omegaRadiansPerSecond;

    this.gyroSim = new SimDeviceSim("Angle", (int) (-Units.radiansToDegrees(chassisRotateSpeed)));
  }

}

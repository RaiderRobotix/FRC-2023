// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro;

public class SwerveWheelController extends SubsystemBase implements Constants {
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private SwerveModuleState[] m_desiredStates;

  private static SwerveWheel frontLeftModule;
  private static SwerveWheel frontRightModule;
  private static SwerveWheel backLeftModule;
  private static SwerveWheel backRightModule;
  private static boolean fieldCentric = true;
  private static boolean coast = false;

  Translation2d frontLeftLocation;
  Translation2d frontRightLocation;
  Translation2d backLeftLocation;
  Translation2d backRightLocation;

  private static SwerveDriveOdometry odometry;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4_L3.getDriveReduction() *
      SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;

  /** Creates a new drivebase. */
  public SwerveWheelController() {
    frontLeftModule = new SwerveWheel(frontLeftDriveID, frontLeftSteerID, frontLeftEncoderID, "Front Left");
    frontRightModule = new SwerveWheel(frontRightDriveID, frontRightSteerID, frontRightEncoderID,"Front Right");
    backLeftModule = new SwerveWheel(backLeftDriveID, backLeftSteerID, backLeftEncoderID, "Back Left");
    backRightModule = new SwerveWheel(backRightDriveID, backRightSteerID, backRightEncoderID, "Back Right");

    m_desiredStates = kDriveKinematics.toSwerveModuleStates(speeds);

    odometry = new SwerveDriveOdometry(
                kDriveKinematics,
                Rotation2d.fromDegrees(Gyro.getHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

    odometry.resetPosition(
      getRotation2d(),
      new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
      new Pose2d(0, 0, getRotation2d()));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    speeds = chassisSpeeds;
    if(Math.abs(m_desiredStates[0].angle.getDegrees() - kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[0].angle.getDegrees()) > 0.5
    || Math.abs(m_desiredStates[1].angle.getDegrees() - kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[1].angle.getDegrees()) > 0.5
    || Math.abs(m_desiredStates[2].angle.getDegrees() - kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[2].angle.getDegrees()) > 0.5
    || Math.abs(m_desiredStates[3].angle.getDegrees() - kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[3].angle.getDegrees()) > 0.5){
      m_desiredStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    }
  }

  public void setState(SwerveModuleState[] states){
    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND, states[3].angle.getRadians());
  }
  
  public static void zeroGyroscope() {
    odometry.resetPosition(
            Rotation2d.fromDegrees(Gyro.getHeading()),
            new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
            new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
    );
    Gyro.resetGyro();
  }

  public Rotation2d getRotation2d(){
    return odometry.getPoseMeters().getRotation();
  }

  public static SwerveDriveOdometry getOdometry(){
    return odometry;
  }

  public void resetOdometry(Pose2d newPose){
    odometry.resetPosition(
      getRotation2d(),
      new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()},
      newPose);
  }

  @Override
  public void periodic() {
    odometry.update(
          Rotation2d.fromDegrees(Gyro
            .getHeading()),
          new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() });

    setState(m_desiredStates);
    
    Shuffleboard.selectTab("Drivetrain");
    SmartDashboard.putNumber("X Point", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y Point", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Rotation Heading", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("X Speed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Angular Speed", speeds.omegaRadiansPerSecond);
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    SmartDashboard.putBoolean("is Coast Mode", coast);
    // This method will be called once per scheduler run
  }
}
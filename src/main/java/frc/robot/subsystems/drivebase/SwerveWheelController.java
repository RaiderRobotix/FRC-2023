// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import java.util.HashMap;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Gyro;

public class SwerveWheelController extends SubsystemBase implements Constants {
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private SwerveModuleState[] m_desiredStates;

  private static Pose2d robotPose;

  private static AHRS ahrs;

  // private Gyro gyro = new Gyro();

  private static SwerveModule frontLeftModule;
  private static SwerveModule frontRightModule;
  private static SwerveModule backLeftModule;
  private static SwerveModule backRightModule;
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

  private static CANCoder frontLeftEncoder = new CANCoder(frontLeftEncoderID);
  private static CANCoder frontRightEncoder = new CANCoder(frontRightEncoderID);
  private static CANCoder backLeftEncoder = new CANCoder(backLeftEncoderID);
  private static CANCoder backRightEncoder = new CANCoder(backRightEncoderID);

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  /** Creates a new drivebase. */
  public SwerveWheelController() {
    Gyro.ahrs = new AHRS(Port.kMXP);
    Gyro.gyro().reset();


    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // Location of modules relative to the centre of the robot
    this.frontLeftLocation = new Translation2d(width / 2, length / 2);
    this.frontRightLocation = new Translation2d(width / 2, -length / 2);
    this.backLeftLocation = new Translation2d(-width / 2, length / 2);
    this.backRightLocation = new Translation2d(-width / 2, -length / 2);
      
    frontLeftModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(5, 6)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L3)
    .withDriveMotor(MotorType.FALCON, frontLeftDriveID)
    .withSteerMotor(MotorType.FALCON, frontLeftSteerID)
    .withSteerEncoderPort(frontLeftEncoderID)
    // .withSteerOffset(frontLeftEncoderOffset)
    .build();

    frontRightModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(5, 6)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L3)
    .withDriveMotor(MotorType.FALCON, frontRightDriveID)
    .withSteerMotor(MotorType.FALCON, frontRightSteerID)
    .withSteerEncoderPort(frontRightEncoderID)
    // .withSteerOffset(frontRightEncoderOffset)
    .build();

    backLeftModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(5, 7)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L3)
    .withDriveMotor(MotorType.FALCON, backLeftDriveID)
    .withSteerMotor(MotorType.FALCON, backLeftSteerID)
    .withSteerEncoderPort(backLeftEncoderID)
    // .withSteerOffset(backLeftEncoderOffset)
    .build();

    backRightModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(5, 6)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L3)
    .withDriveMotor(MotorType.FALCON, backRightDriveID)
    .withSteerMotor(MotorType.FALCON, backRightSteerID)
    .withSteerEncoderPort(backRightEncoderID)
    // .withSteerOffset(backRightEncoderOffset)
    .build();

    

    m_desiredStates = kDriveKinematics.toSwerveModuleStates(speeds);

    odometry = new SwerveDriveOdometry(
                kDriveKinematics,
                Rotation2d.fromDegrees(frc.robot.subsystems.Gyro
            .getHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

    odometry.resetPosition(
      getRotation2d(),
      new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
      new Pose2d(0, 0, getRotation2d()));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    speeds = chassisSpeeds;
    m_desiredStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    if(Math.abs(m_desiredStates[0].angle.getDegrees() - kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[0].angle.getDegrees()) > 0.5
    || Math.abs(m_desiredStates[1].angle.getDegrees() - kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[1].angle.getDegrees()) > 0.5
    || Math.abs(m_desiredStates[2].angle.getDegrees() - kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[2].angle.getDegrees()) > 0.5
    || Math.abs(m_desiredStates[3].angle.getDegrees() - kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[3].angle.getDegrees()) > 0.5){
      m_desiredStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    }
  }

  public void setState(SwerveModuleState[] states){
    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians() + Units.degreesToRadians(frontLeftEncoderOffset));
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians() + Units.degreesToRadians(frontRightEncoderOffset));
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians() + Units.degreesToRadians(backLeftEncoderOffset));
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians() + Units.degreesToRadians(backRightEncoderOffset));
  }
  
  public static void zeroGyroscope() {
    odometry.resetPosition(
            Rotation2d.fromDegrees(frc.robot.subsystems.Gyro.getHeading()),
            new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
            new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
    );
    // Gyro.gyro().reset();
  }

  public Rotation2d getRotation2d(){
    return odometry.getPoseMeters().getRotation();
  }

  public SwerveDriveOdometry getOdometry(){
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
          Rotation2d.fromDegrees(frc.robot.subsystems.Gyro
            .getHeading()),
          new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() });

    setState(m_desiredStates);
    
    Shuffleboard.selectTab("Drivetrain");
    SmartDashboard.putNumber("X Point", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y Point", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("X Speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Angular Speed", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    SmartDashboard.putBoolean("is Coast Mode", coast);
    // This method will be called once per scheduler run
  }
}
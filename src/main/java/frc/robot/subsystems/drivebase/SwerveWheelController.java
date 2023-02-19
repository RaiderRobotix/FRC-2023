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
      SdsModuleConfigurations.MK4_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  private static CANCoder frontLeftEncoder = new CANCoder(frontLeftEncoderID);
  private static CANCoder frontRightEncoder = new CANCoder(frontRightEncoderID);
  private static CANCoder backLeftEncoder = new CANCoder(backLeftEncoderID);
  private static CANCoder backRightEncoder = new CANCoder(backRightEncoderID);

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  /** Creates a new drivebase. */
  public SwerveWheelController() {

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // Location of modules relative to the centre of the robot
    this.frontLeftLocation = new Translation2d(width / 2, length / 2);
    this.frontRightLocation = new Translation2d(width / 2, -length / 2);
    this.backLeftLocation = new Translation2d(-width / 2, length / 2);
    this.backRightLocation = new Translation2d(-width / 2, -length / 2);
      
    frontLeftModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L3)
    .withDriveMotor(MotorType.FALCON, frontLeftDriveID)
    .withSteerMotor(MotorType.FALCON, frontLeftSteerID)
    .withSteerEncoderPort(frontLeftEncoderID)
    .withSteerOffset(frontLeftEncoderOffset)
    .build();

    frontRightModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L3)
    .withDriveMotor(MotorType.FALCON, frontRightDriveID)
    .withSteerMotor(MotorType.FALCON, frontRightSteerID)
    .withSteerEncoderPort(frontRightEncoderID)
    .withSteerOffset(frontRightEncoderOffset)
    .build();

    backLeftModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L3)
    .withDriveMotor(MotorType.FALCON, backLeftDriveID)
    .withSteerMotor(MotorType.FALCON, backLeftSteerID)
    .withSteerEncoderPort(backLeftEncoderID)
    .withSteerOffset(backLeftEncoderOffset)
    .build();

    backRightModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L3)
    .withDriveMotor(MotorType.FALCON, backRightDriveID)
    .withSteerMotor(MotorType.FALCON, backRightSteerID)
    .withSteerEncoderPort(backRightEncoderID)
    .withSteerOffset(backRightEncoderOffset)
    .build();

    

    m_desiredStates = kDriveKinematics.toSwerveModuleStates(speeds);

    odometry = new SwerveDriveOdometry(
                kDriveKinematics,
                Rotation2d.fromDegrees(Gyro.gyro().getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    speeds = chassisSpeeds;
    m_desiredStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  }

  public static void toggleField() {
    fieldCentric ^= true;
  }

  public static void toggleCoast() {
    coast ^= true;
  }
  
  public void zeroGyroscope() {
    odometry.resetPosition(
            Rotation2d.fromDegrees(Gyro.gyro().getFusedHeading()),
            new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
            new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
    );
  }

  public Rotation2d getRotation2d(){
    return odometry.getPoseMeters().getRotation();
  }

  @Override
  public void periodic() {
    odometry.getPoseMeters().getTranslation().getDistance(backLeftLocation)
    odometry.update(
          Rotation2d.fromDegrees(Gyro.gyro().getFusedHeading()),
          new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() });

    SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    SmartDashboard.putNumber("X Speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Angular Speed", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    SmartDashboard.putBoolean("is Coast Mode", coast);
    // This method will be called once per scheduler run
  }
}

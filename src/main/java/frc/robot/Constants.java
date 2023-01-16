// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {
    public final int xboxControllerPort = 4;

    // Falcon 500 constants
    public final int kUnitsPerRevoltion = 2048;
    public final double kGearRatio = 6.12;
    public final int kMaxRPM = 6380;
    public final double kWheelRadius = 1.5;

    public final double rightDeadband = 0.15;
    public final double leftDeadband = 0.15;
    public final int leftBumberId = 5;
    public final int rightBumberId = 6;

    // Robots measurements in metres
    public final double width = .63;
    public final double length = .79;

    //Robot's kinematics
    public final double frontLeftLocationX = 1.0;
    public final double frontLeftLocationY = 1.0;

    public final double frontRightLocationX = 1.0;
    public final double frontRightLocationY = 1.0;

    public final double backLeftLocationX = 1.0;
    public final double backLeftLocationY = 1.0;

    public final double backRightLocationX = 1.0;
    public final double backRightLocationY = 1.0;

    public final SwerveDriveKinematics kDriveKinematics = 
        new SwerveDriveKinematics(
            new Translation2d(width / 2, length / 2),
            new Translation2d(width / 2, -length / 2),
            new Translation2d(-width / 2, length / 2),
            new Translation2d(-width / 2, -length / 2)
        );


    //TODO find what the units are
    //Drivebase movement constraints units are in 
    public final double kPhysicalDriveMaxSpeed = 5;
    public final double maxAttainableSpeed = 18.0;
    public final double kPhysicalSteerMaxSpeed = 5;
    public final double rotateSpeed = 0.5;
    public final double kMaxAccelerationMetersPerSecondSquared = 5;





    //PID values
    
    //PID values for motors
    //PID values for steering angle
    public final double angleKp = 0.01120; // Speed Default 0.01100
    public final double angleKi = 0.0000001; // Amount to react Default 0.00001
    public final double angleKd = 0.00023; // Dampens system Default 0.00037

    //PID values for driving speed
    public final double driveKp = 500;
    public final double driveKi = 0.055;
    public final double driveKd = 0.00;
    
    //PID values for commands
    //PID values for robot angle/heading
    public final double robotangleKi = 0.00100; // Amount to react Default 0.00001
    public final double robotangleKp = 0.00000; // Speed Default 0.01100
    public final double robotangleKd = 0.00000; // Dampens system Default 0.00037

    //PID values for robot drive distance
    public final double robotDriveDistanceKi = 0.0000;
    public final double robotDriveDistanceKp = 0.0000;
    public final double robotDriveDistanceKd = 0.0000;

    //PID values for thete controller
    public final double thetaControllerKp = 0.0000;
    public final double thetaControllerKi = 0.0000;
    public final double thetaControllerKd = 0.0000;

    //PID values for position controller
    //X value
    public final double xControllerKp = 0.0000;
    public final double xControllerKi = 0.0000; //Dont use
    public final double xControllerKd = 0.0000; //Dont
    //Y Value
    public final double yControllerKp = 0.0000;
    public final double yControllerKi = 0.0000; //Dont use
    public final double yControllerKd = 0.0000; //Dont use




    //CAN IDs and Encoder Offsets
    public final int frontLeftDriveID = 20;
    public final int frontLeftSteerID = 10;
    public final int frontLeftEncoderID = 0;
    public final double frontLeftEncoderOffset = -18;

    public final int frontRightDriveID = 21;
    public final int frontRightSteerID = 11;
    public final int frontRightEncoderID = 1;
    public final double frontRightEncoderOffset = -10.61;

    public final int backLeftDriveID = 22;
    public final int backLeftSteerID = 12;
    public final int backLeftEncoderID = 2;
    public final double backleftEncoderOffset = 280.9;

    public final int backRightDriveID = 23;
    public final int backRightSteerID = 13;
    public final int backRightEncoderID = 3;
    public final double backRightEncoderOffset = 2.59;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraint = new TrapezoidProfile.Constraints(
        kPhysicalDriveMaxSpeed, kPhysicalSteerMaxSpeed);


}
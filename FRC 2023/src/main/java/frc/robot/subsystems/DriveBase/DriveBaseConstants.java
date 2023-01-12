package frc.robot.subsystems.DriveBase;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface DriveBaseConstants {
    // TODO find values
    // Swerve Drive Wheel Locations
    public final double frontLeftLocationX = 1.0;
    public final double frontLeftLocationY = 1.0;

    public final double frontRightLocationX = 1.0;
    public final double frontRightLocationY = 1.0;

    public final double backLeftLocationX = 1.0;
    public final double backLeftLocationY = 1.0;

    public final double backRightLocationX = 1.0;
    public final double backRightLocationY = 1.0;

    public final double kPhysicalDriveMaxSpeed = 5;
    public final double kPhysicalSteerMaxSpeed = 5;
    public final double rotateSpeed = 0.5;

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

    // Set kp, ki, and kd to zero.

    // Increase kp until the output starts to oscillate around the setpoint.

    // Increase kd as much as possible without introducing jittering in the system
    // response.

    // Increase ki

    public final double angleKp = 0.01120; // Speed Default 0.01100
    public final double angleKi = 0.0000001; // Amount to react Default 0.00001
    public final double angleKd = 0.00023; // Dampens system Default 0.00037

    public final double robotangleKi = 0.00100; // Amount to react Default 0.00001
    public final double robotangleKp = 0.00000; // Speed Default 0.01100
    public final double robotangleKd = 0.00000; // Dampens system Default 0.00037

    public final double driveKp = 500;
    public final double driveKi = 0.055;
    public final double driveKd = 0.00;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraint = new TrapezoidProfile.Constraints(
            kPhysicalDriveMaxSpeed, kPhysicalSteerMaxSpeed);
}
package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface drivebaseConstants {
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

    public final double kPhysicalDriveMaxSpeed = 1;
    public final double kPhysicalSteerMaxSpeed = 5;
    public final double rotateSpeed = 5;

    public final int frontLeftDriveID = 20;
    public final int frontLeftSteerID = 10;
    public final int frontLeftEncoderID = 0;

    public final int frontRightDriveID = 21;
    public final int frontRightSteerID = 11;
    public final int frontRightEncoderID = 1;

    public final int backLeftDriveID = 22;
    public final int backLeftSteerID = 12;
    public final int backLeftEncoderID = 2;

    public final int backRightDriveID = 23;
    public final int backRightSteerID = 13;
    public final int backRightEncoderID = 3;

    // Set kp, ki, and kd to zero.

    // Increase kp until the output starts to oscillate around the setpoint.

    // Increase kd as much as possible without introducing jittering in the system
    // response.

    // Increase ki

    public final double angleKp = 0.0100;
    public final double angleKi = 0.0015;
    public final double angleKd = 0.0001;

    public final double driveKp = 875;
    public final double driveKi = 0.06;
    public final double driveKd = 0.01;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraint = new TrapezoidProfile.Constraints(
            kPhysicalDriveMaxSpeed, kPhysicalSteerMaxSpeed);
}

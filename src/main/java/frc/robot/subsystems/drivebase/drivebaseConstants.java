package frc.robot.subsystems.drivebase;

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

    public final double kPhysicalDriveMaxSpeed = 5.0;

    public final int frontLeftDriveID = 0;
    public final int frontLeftSteerID = 1;
    public final int frontLeftEncoderID = 12;

    public final int frontRightDriveID = 2;
    public final int frontRightSteerID = 3;
    public final int frontRightEncoderID = 10;

    public final int backLeftDriveID = 6;
    public final int backLeftSteerID = 7;
    public final int backLeftEncoderID = 11;

    public final int backRightDriveID = 5;
    public final int backRightSteerID = 4;
    public final int backRightEncoderID = 13;

    // Set kp, ki, and kd to zero.

    // Increase kp until the output starts to oscillate around the setpoint.

    // Increase kd as much as possible without introducing jittering in the system
    // response.

    // Increase ki

    public final double angleKp = 0.5;
    public final double angleKi = 0;
    public final double angleKd = 0;
}

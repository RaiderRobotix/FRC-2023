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

    // Set kp, ki, and kd to zero.

    // Increase kp until the output starts to oscillate around the setpoint.

    // Increase kd as much as possible without introducing jittering in the system
    // response.

    // Increase ki

    public final double angleKp = 0;
    public final double angleKi = 0;
    public final double angleKd = 0;
    public final double driveKp = 0;
    public final double driveKi = 0;
    public final double driveKd = 0;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

    // Driver constants
    public final int xboxControllerPort = 4;

    // Operator Constants
    public final int operatorStickPort = 1;
    public final int grabberJoystickButton = 1;

    public final int armInJoystickButton = 11;
    public final int armOutJoystickButton = 12;

    public final int elevatorUpJoystickButton = 5;
    public final int elevatorDownJoystickButton = 3;

    //Distance Sensor Constants
    public final int kGrabberDistanceSensorDIO = 0;
    //Time between the sensor allowing to trigger the grabber
    //in seconds
    public final double kDistanceSensorDebounceTime = 30;

    // Falcon 500 constants
    public final int kUnitsPerRevoltion = 2048;
    public final double kGearRatio = 6.12;
    public final int kMaxRPM = 6380;
    public final double kWheelDiameter = Units.inchesToMeters(4);

    public final double rightDeadband = 0.15;
    public final double leftDeadband = 0.15;
    public final double rightTriggerThreshold = 0.70;
    public final double leftTriggerThreshold = 0.70;






    // Robots measurements in metres
    public final double width = Units.inchesToMeters(24);
    public final double length = Units.inchesToMeters(32);





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


    //Drivebase movement constraints units are in metres
    public final double maxAttainableSpeed = 18.0;
    //Units in percentage of maxAttainableSpeed
    public final double turboSpeed = 1;
    public final double slowSpeed = 0.2;

    //Auton Path trajectory thing ---- STILL IN TESTING
    //ArrayList<PathPlannerTrajectory> pathGroup = new ArrayList<PathPlannerTrajectory>();


    //Drivebase movement constraints for auton
    public final double kPhysicalDriveMaxSpeed = 1;
    public final double kPhysicalSteerMaxSpeed = 5;
    public final double kMaxAccelerationMetersPerSecondSquared = 0.5;


    //Pneumatics Constansts
    public final double maxPSI = 80.0;
    public final double minPSI = 0.0;

    // Solenoid Channel Values

    //Channels for solenoid that grabs the game pieces
    public final int grabberSolenoidOnChannel = 13;
    public final int grabberSolenoidOffChannel = 2;

    //Channels for solenoids that pop game piece in auton
    public final int popperSolenoidOnChannel = 12;
    public final int popperSolenoidOffChannel = 1;

    //Elevator Variables
    public final int kElevatorEncoder = 1;
    public final int kElevatorTalonFX = 1;
    public final double kElevatorDistancePerRotation = 0.0;

    //Speeds for Operator Controller
    public final double kElevatorUpSpeed = 0.1;
    public final double kElevatorDownSpeed = 0.1;

    //Present Height Values for rows
    public final double kUpperRowHeight = 0.0;
    public final double kMidRowHeight = 0.0;
    public final double kLowerRowHeight = 0.0;
    //PID values for elevator
    public final double elevatorKp = 0.0100;
    public final double elevatorKi = 0.0000;
    public final double elevatorKd = 0.0000;


    //Arm Variables
    public final int kArmEncoder = 0;
    public final int kArmTalonFX = 0;
    public final double kArmDistancePerRotation = 0.0;

    //Speeds for Operator Arm
    public final double kArmInSpeed = 0.1;
    public final double kArmOutSpeed = 0.1;
    
     //PID values for Arm
     public final double armKp = 0.0100;
     public final double armKi = 0.0000;
     public final double armKd = 0.0000;



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
    public final double robotAngleTolerance = 1.5;

    //PID values for robot drive distance
    public final double robotDriveDistanceKi = 0.2000;
    public final double robotDriveDistanceKp = 0.0000;
    public final double robotDriveDistanceKd = 0.0000;
    public final double robotDistanceTolerance = .1;

    //PID values for thete controller
    public final double thetaControllerKp = 0.5000;
    public final double thetaControllerKi = 0.0000;
    public final double thetaControllerKd = 0.0000;

    //PID values for position controller
    //X value
    public final double xControllerKp = 0.5000;
    public final double xControllerKi = 0.1100; //Dont use
    public final double xControllerKd = 0.0000; //Dont
    //Y Value
    public final double yControllerKp = 0.5000;
    public final double yControllerKi = 0.0000; //Dont use
    public final double yControllerKd = 0.0000; //Dont use

    //PID values for robot balance command
    public final double balanceKp = 0.0000;
    public final double balanceKi = 0.0000;
    public final double balanceKd = 0.0000;
    public final double robotBalanceTolerance = 1;   

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
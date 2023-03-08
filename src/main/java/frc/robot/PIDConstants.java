// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

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

 public final class PIDConstants {
    public static class robotAngle {
        //PID values for robot angle/heading
        public static double ki = 0.00100; // Amount to react Default 0.00001
        public static double kp = 0.00000; // Speed Default 0.01100
        public static double kd = 0.00000; // Dampens system Default 0.00037
        public static double tolerance = 1.5;
    }

    public static class robotDriveDistance {
        //PID values for robot drive distance
        public static double robotDriveDistanceKi = 0.0050;
        public static double robotDriveDistanceKp = 0.0000;
        public static double robotDriveDistanceKd = 0.0000;
        public static double robotDistanceTolerance = .1;
    }

    public static class robotXPosition {
        //X value
        public static double xControllerKp = 0.5000;
        public static double xControllerKi = 0.1100; //Dont use
        public static double xControllerKd = 0.0000; //Dont
    }

    public static class robotYPosition {
        //Y Value
        public static double yControllerKp = 0.5000;
        public static double yControllerKi = 0.0000; //Dont use
        public static double yControllerKd = 0.0000; //Dont use
    }

    public static class balanceCommand {
        //PID values for robot balance command
        public static double kp = 0.0000;
        public static double ki = 0.0000;
        public static double kd = 0.0000;
        public static double tolerance = 1;   
    }
 }
package frc.robot.subsystems.DriveBase;


import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;



public class driveConstants {

    public static final int LEFT_FRONT_DRIVE = 1;
    public static final int LEFT_BACK_DRIVE = 2;
    public static final int RIGHT_FRONT_DRIVE = 4;
    public static final int RIGHT_BACK_DRIVE = 3;

    public static final TalonFX LEFT_FRONT_TALONFX = new TalonFX(LEFT_FRONT_DRIVE);
    public static final TalonFX LEFT_BACK_TALONFX = new TalonFX(LEFT_BACK_DRIVE);
    public static final TalonFX RIGHT_FRONT_TALONFX = new TalonFX(RIGHT_FRONT_DRIVE);
    public static final TalonFX RIGHT_BACK_TALONFX = new TalonFX(RIGHT_BACK_DRIVE);

    public static final TalonFX[] Left = {LEFT_FRONT_TALONFX,LEFT_BACK_TALONFX};
    public static final TalonFX[] Right = {RIGHT_FRONT_TALONFX,RIGHT_BACK_TALONFX};
    
    public static final MotorControllerGroup left_group = new MotorControllerGroup((MotorController[]) Left);
    public static final MotorControllerGroup right_group = new MotorControllerGroup((MotorController[]) Right);


    public static final DifferentialDrive driveSystem = new DifferentialDrive(left_group, right_group);



}

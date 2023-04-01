package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private Joystick driver;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private double maxSpeed;

    public TeleopSwerve(Swerve s_Swerve, Joystick driver, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        
        this.driver = driver;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.maxSpeed = 0.6;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        if(driver.getRawAxis(2) >= .70){
            maxSpeed = 0.22;
        } else if (driver.getRawAxis(3) >= .70){
            maxSpeed = 1;
        } else {
            maxSpeed = 0.6;
        }
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * maxSpeed;
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * maxSpeed;
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * 2.5*maxSpeed;

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed), 
            rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );

        SmartDashboard.putNumber("Left Joystick Y: ", translationSup.getAsDouble());
    }
}
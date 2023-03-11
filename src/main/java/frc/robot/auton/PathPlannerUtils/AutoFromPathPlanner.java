package frc.robot.auton.PathPlannerUtils;

import java.lang.String;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoFromPathPlanner extends SequentialCommandGroup {
  private PathPlannerTrajectory m_trajectory;

  public AutoFromPathPlanner(Swerve s_swerve, String pathName, double maxSpeed, boolean endStationary) {
    m_trajectory = PathPlanner.loadPath(pathName, maxSpeed, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    //Changed from -Math.PI, Math.PI
    thetaController.enableContinuousInput(0, 360);

    AutoSwerveController swerveControllerCommand = new AutoSwerveController(m_trajectory, s_swerve::getPose,
        Constants.SwerveConstants.swerveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController, s_swerve::setModuleStates, s_swerve);

    addRequirements(s_swerve);

    if(endStationary){
    // Run path following command, then stop at the end.
      addCommands(
         new InstantCommand(() -> SmartDashboard.putString("AutoPath", pathName)),
          swerveControllerCommand, 
          new InstantCommand(() -> s_swerve.stop()));
    }
    else{
          // Run path following command, robot will continue moving at same swerve module commanded state at the end 
          // (NEXT COMMAND IN AUTO MUST BE MOVEMENT OR ELSE ROBOT MAY CONTINUE MOVING UNCONTROLLABLY)
          addCommands(new InstantCommand(() -> SmartDashboard.putString("AutoPath", pathName)),
          swerveControllerCommand);
    }


  }

  public Pose2d getInitialPose() {
    return new Pose2d(m_trajectory.getInitialState().poseMeters.getX(),
        m_trajectory.getInitialState().poseMeters.getY(),
        m_trajectory.getInitialState().holonomicRotation.times(-1.0));
  }
}

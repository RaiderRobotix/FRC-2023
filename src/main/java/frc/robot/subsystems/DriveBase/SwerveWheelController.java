// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveBase;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro;

public class SwerveWheelController extends SubsystemBase implements Constants {

  private ChassisSpeeds speeds = new ChassisSpeeds();
  private SwerveModuleState[] m_desiredStates;


  // private Gyro gyro = new Gyro();

  private static SwerveModule frontLeftModule;
  private static SwerveModule frontRightModule;
  private static SwerveModule backLeftModule;
  private static SwerveModule backRightModule;
  private static boolean fieldCentric = true;
  private static boolean coast = false;

  Translation2d frontLeftLocation;
  Translation2d frontRightLocation;
  Translation2d backLeftLocation;
  Translation2d backRightLocation;

  private static SwerveDriveOdometry odometry;

  public static final double MAX_VOLTAGE = 12.0;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
  SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  private CANCoder frontLeftEncoder = new CANCoder(frontLeftEncoderID);
  private CANCoder frontRightEncoder = new CANCoder(frontRightEncoderID);
  private CANCoder backLeftEncoder = new CANCoder(backLeftEncoderID);
  private CANCoder backRightEncoder = new CANCoder(backRightEncoderID);
  
  // public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
  // Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private PIDController angleController = new PIDController(robotangleKd, robotangleKi,
      robotangleKd);

  public AHRS gyro;

  private static SwerveModulePosition[] driveModules = new SwerveModulePosition[]{};  
  
  //private static SwerveWheel[] modules =  new SwerveWheel[4];

  /** Creates a new drivebase. */
  public SwerveWheelController() {

    gyro = new AHRS(Port.kMXP);

    // Location of modules relative to the centre of the robot
    this.frontLeftLocation = new Translation2d(width / 2, length / 2);
    this.frontRightLocation = new Translation2d(width / 2, -length / 2);
    this.backLeftLocation = new Translation2d(-width / 2, length / 2);
    this.backRightLocation = new Translation2d(-width / 2, -length / 2);

    frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
      Mk4iSwerveModuleHelper.GearRatio.L3,
      frontLeftDriveID,
      frontLeftSteerID,
      frontLeftEncoderID,
      frontLeftEncoderOffset);

    frontRightModule = Mk4iSwerveModuleHelper.createNeo(
      Mk4iSwerveModuleHelper.GearRatio.L3,
      frontRightDriveID,
      frontRightSteerID,
      frontRightEncoderID,
      frontRightEncoderOffset);

    backLeftModule = Mk4iSwerveModuleHelper.createNeo(
      Mk4iSwerveModuleHelper.GearRatio.L3,
      backLeftDriveID,
      backLeftSteerID,
      backLeftEncoderID,
      backleftEncoderOffset);

    backRightModule = Mk4iSwerveModuleHelper.createNeo(
      Mk4iSwerveModuleHelper.GearRatio.L3,
      backRightDriveID,
      backRightSteerID,
      backRightEncoderID,
      backRightEncoderOffset);

    m_desiredStates = kDriveKinematics.toSwerveModuleStates(speeds);

    this.odometry = new SwerveDriveOdometry(kDriveKinematics, 
    getRotation2d(),
    new SwerveModulePosition[]{getPosition(0), getPosition(1), getPosition(2), getPosition(3)},
    new Pose2d());


    angleController.enableContinuousInput(0, 360);
    // reset();
  }

  private void getPostition(int i, SwerveModulePosition position, SwerveModulePosition position2,
      SwerveModulePosition position3, Pose2d pose2d) {
  }

  public static void reset(){
    resetMotors();
    resetSteerMotors();
    resetOdometry(new Pose2d());
    Gyro.gyro().reset();
    Gyro.gyro().calibrate();
  }
  public static void resetMotors() {
    // for(SwerveModule module : modules){
    //   module.resetEncoders();
    // }

    
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    speeds = chassisSpeeds;
    m_desiredStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  }

  //Resets the CANcoder angles to 0
  public static void resetSteerMotors() {
    // for(SwerveWheel module : modules){
    //   module.resetSteerMotor();
    // }
  }
  
  public static void zeroGyroscope(){
    Gyro.gyro().zeroYaw(); 
  }

  
  public static void toggleField() {
    fieldCentric ^= true;
  }

  public static void stopMotors() {
    // for(SwerveWheel module : modules){
    //   module.stop();
    // }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public static void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), driveModules, pose);
  }

  public static void toggleCoast() {
    coast ^= true;
    
  }

  public void setSpeed(double x, double y, double delta) {
    SmartDashboard.putNumber("DesiredXSpeed", x);
    SmartDashboard.putNumber("DesiredYSpeed", y);
    SmartDashboard.putBoolean("Field Centric", fieldCentric );
    if (this.fieldCentric) {
      this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());
    } else {
      this.speeds = new ChassisSpeeds(x, y, delta);
    }
    
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setState(moduleStates);
  }

  public void setSpeed(double speed, boolean sideToside, boolean fieldCentric) {
    double x = 0;
    double y = 0;
    double delta = 0;
    if(sideToside){
      x = speed;
    } else {
      y = speed;
    }
    if (fieldCentric) {
      this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());
    } else {
      this.speeds = new ChassisSpeeds(x, y, delta);
    }
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setState(moduleStates);
  }

  public void setSpeed(double x, double y, double delta, boolean fieldCentric) {
    SmartDashboard.putNumber("DesiredXSpeed", x);
    SmartDashboard.putNumber("DesiredYSpeed", y);
    if (fieldCentric) {
      this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, delta, getRotation2d());
    } else {
      this.speeds = new ChassisSpeeds(x, y, delta);
    }
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setState(moduleStates);
  }

  public void setState(SwerveModuleState[] moduleStates) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxAttainableSpeed);
    // frontLeftModule.setDesiredState(moduleStates[0]);
    // frontRightModule.setDesiredState(moduleStates[1]);
    // backLeftModule.setDesiredState(moduleStates[2]);
    // backRightModule.setDesiredState(moduleStates[3]);

      kDriveKinematics.toChassisSpeeds(moduleStates);
  }

  public void setHeading(double angle, double maxSpeed) {
    setSpeed(0, 0, angleController.calculate(Gyro.getHeading(), angle));
  }

  // Sets the angle of all the wheel to angle
  public static void setAngle(double angle) {
    // for(SwerveWheel module : modules){
    //   module.setSteerAngle(angle);
    // }
  }

  public double getXSpeed() {
    if (this.speeds != null) {
      return this.speeds.vxMetersPerSecond;
    } else {
      return 0;
    }
  }

  public void setSteerZero() {
    // for(SwerveWheel module : modules){
    //   module.absoluteAngle();
    // }
  }

  public double getYSpeed() {
    if (this.speeds != null) {
      return this.speeds.vyMetersPerSecond;
    } else {
      return 0;
    }
  }

  public SwerveModulePosition getPosition(int moduleNum){
    double frontLeftPos = frontLeftEncoder.getPosition();
    double frontRightPos = frontRightEncoder.getPosition();
    double backLeftPos = backLeftEncoder.getPosition();
    double backRightPos =  backRightEncoder.getPosition();

    switch(moduleNum){
      case 1:
        return new SwerveModulePosition(frontLeftPos * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()/((6.75) * 4096), new Rotation2d(frontLeftEncoder.getAbsolutePosition()*Math.PI/180));
      case 2:
        return new SwerveModulePosition(frontRightPos * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()/((6.75) * 4096), new Rotation2d(frontRightEncoder.getAbsolutePosition()*Math.PI/180));
      case 3:
        return new SwerveModulePosition(backLeftPos * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()/((6.75) * 4096), new Rotation2d(backLeftEncoder.getAbsolutePosition()*Math.PI/180));
      case 4:
        return new SwerveModulePosition(backRightPos * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()/((6.75) * 4096), new Rotation2d(backRightEncoder.getAbsolutePosition()*Math.PI/180));
      default:
        return new SwerveModulePosition();
    }
  }

  public Rotation2d getGyroscopeRotation(){
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void updateOdometry()
{
        odometry.update(Rotation2d.fromDegrees(getGyroscopeRotation().getDegrees()), new SwerveModulePosition[]{frontLeftPos(), frontRightPos(), backLeftPos(), backRightPos()});
}

public SwerveModulePosition frontLeftPos()
{
        return getPosition(4);
}

public SwerveModulePosition frontRightPos()
{
        return getPosition(1);
}

public SwerveModulePosition backLeftPos()
{
        return getPosition(3);
}

public SwerveModulePosition backRightPos()
{
        return getPosition(2);
}

  public ChassisSpeeds getCurrentSpeed()
{
    SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(speeds);
    //SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

        ChassisSpeeds speed;
        speed = kDriveKinematics.toChassisSpeeds(states);
        return speed;
}

  public double getAngularSpeed() {
    if (this.speeds != null) {
      return this.speeds.omegaRadiansPerSecond;

    }
    return 0;
  }

  public double getHeading() {
    return Gyro.gyro().getYaw();
  }

  public static Rotation2d getRotation2d() {
    // new Rotation2d();
    
    if (Gyro.gyro() == null) {
      return new Rotation2d().fromDegrees(0);
    } else {
      // return Rotation2d.fromDegrees(Gyro.gyro().getYaw());
      return new Rotation2d().fromDegrees(Gyro.getHeading());
    }
    // return Rotation2d.fromDegrees(Gyro.gyro().getCompassHeading());
  }

  public double getSteerAngle(){
    return frontLeftModule.getSteerAngle();
  }

  public double getDistance(){
    return frontLeftModule.getDriveVelocity();
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(),driveModules);

    SmartDashboard.putNumber("X Speed", getXSpeed());
    SmartDashboard.putNumber("Y Speed", getYSpeed());
    SmartDashboard.putNumber("Angular Speed", getAngularSpeed());
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    SmartDashboard.putBoolean("is Coast Mode", coast);
    // SmartDashboard.putNumber("Compass Angle", Gyro.gyro().getRotation2d().getDegrees());

    SmartDashboard.updateValues();
    // This method will be called once per scheduler run
  }
}

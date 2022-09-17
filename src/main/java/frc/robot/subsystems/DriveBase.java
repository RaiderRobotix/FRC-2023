// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.spec.EncodedKeySpec;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  /** Creates a new DriveBase. */
  private static DriveBase m_instance;

  public final WPI_TalonFX leftFrontMotor;
  private final WPI_TalonFX leftBackMotor;
  private final WPI_TalonFX rightFrontMotor;
  private final WPI_TalonFX rightBackMotor;

  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;

  AnalogGyro gyro;

  private AnalogGyroSim m_gyroSim;

  private final DifferentialDrivetrainSim m_driveSim;

  private DifferentialDriveOdometry odometry;

  private Field2d m_field = new Field2d();

  private OperatorInterface oi;

  public DriveBase() {

    this.oi = new OperatorInterface();

    SmartDashboard.putData("Field", m_field);
    this.leftFrontMotor = new WPI_TalonFX(0);
    this.leftBackMotor = new WPI_TalonFX(1);
    this.rightFrontMotor = new WPI_TalonFX(2);
    this.rightBackMotor = new WPI_TalonFX(3);

    this.leftBackMotor.follow(this.leftFrontMotor);
    this.rightBackMotor.follow(this.rightFrontMotor);

    this.leftEncoder = new Encoder(10, 11);
    this.rightEncoder = new Encoder(12, 13);

    this.leftEncoderSim = new EncoderSim(leftEncoder);
    this.rightEncoderSim = new EncoderSim(rightEncoder);

    gyro = new AnalogGyro(0);

    this.m_gyroSim = new AnalogGyroSim(this.gyro);

    this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), new Pose2d(5.0, 5.0, new Rotation2d()));

    this.m_driveSim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), 7.29, 3, 60.0, Units.inchesToMeters(3),
        0.7112, null);
  }

  public void setSpeed(double speedLeft, double speedRight) {
    leftFrontMotor.set(speedLeft);
    rightFrontMotor.set(speedRight);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance());

    m_field.setRobotPose(odometry.getPoseMeters());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // System.out.println(this.leftFrontMotor.get());
    // m_driveSim.setInputs(.2 * RobotController.getInputVoltage(),
    // .3 * RobotController.getInputVoltage());
    // m_driveSim.setInputs(oi.getLeftY() * -RobotController.getInputVoltage(),
    // oi.getRightY() * -RobotController.getInputVoltage());
    m_driveSim.setInputs(this.leftFrontMotor.get() *
        RobotController.getInputVoltage(),
        this.rightFrontMotor.get() *
            RobotController.getInputVoltage());
    m_driveSim.update(0.02);

    if (oi.getLeftButton(1)) {
    }

    leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());

    rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle((-m_driveSim.getHeading().getDegrees()) % 360);

  }
}

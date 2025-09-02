// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  SparkMax motorFrontLeft = new SparkMax (Constants.DrivetrainConstants.frontLeftCANID, SparkLowLevel.MotorType.kBrushless);
  SparkMax motorFrontRight = new SparkMax (Constants.DrivetrainConstants.frontRightCANID, SparkLowLevel.MotorType.kBrushless);
  SparkMax motorBackLeft = new SparkMax (Constants.DrivetrainConstants.backLeftCANID, SparkLowLevel.MotorType.kBrushless);
  SparkMax motorBackRight = new SparkMax (Constants.DrivetrainConstants.backRightCANID, SparkLowLevel.MotorType.kBrushless);
  
  RelativeEncoder encoderLeft = motorFrontLeft.getEncoder();
  RelativeEncoder encoderRight = motorFrontRight.getEncoder();

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(motorFrontLeft, motorBackLeft);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(motorFrontRight, motorBackRight);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  public static final AHRS navX = new AHRS(NavXComType.kMXP_SPI);
  private final DifferentialDriveOdometry m_Odometry;

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    SparkMaxConfig backLeftConfig = new SparkMaxConfig();
    SparkMaxConfig backRightConfig = new SparkMaxConfig();

    motorFrontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorBackLeft.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFrontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorBackRight.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
    
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);

    SparkMaxConfig encoderConfig = new SparkMaxConfig();
    encoderConfig.encoder
      .positionConversionFactor((Units.inchesToMeters(1/(12.6*2*Math.PI*Units.inchesToMeters(3))*10)))
      .velocityConversionFactor((Units.inchesToMeters(1/(12.6*2*Math.PI*Units.inchesToMeters(3))*10))/60);
    motorFrontLeft.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFrontRight.configure(encoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    backLeftConfig.follow(motorFrontLeft);
    backRightConfig.follow(motorFrontRight);

    leftControllerGroup.setInverted(false);
    rightControllerGroup.setInverted(true);

    navX.reset();
    navX.isCalibrating();
    resetEncoders();

    m_Odometry = new DifferentialDriveOdometry(navX.getRotation2d(), null, null);
    m_Odometry.resetPosition(navX.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition(), new Pose2d());
  }

  public void setBrakeMode() {
    SparkBaseConfig brakeConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake);
      motorFrontLeft.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motorBackLeft.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motorFrontRight.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motorBackRight.configure(brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setCoastMode() {
    SparkBaseConfig coastConfig = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast);
      motorFrontLeft.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motorBackLeft.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motorFrontRight.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motorBackRight.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void resetEncoders() {
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public double getLeftEncoderPosition() {
    return -encoderLeft.getPosition();
  }

  public double getRightEncoderPosition() {
    return encoderRight.getPosition();
  }

  public double getLeftEncoderVelocity() {
    return -encoderLeft.getVelocity();
  }

  public double getRightEncoderVelocity() {
    return encoderRight.getVelocity();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public static double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_Odometry.resetPosition(navX.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getLeftEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllerGroup.setVoltage(leftVolts);
    rightControllerGroup.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition()*getRightEncoderPosition())/2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return encoderLeft;
  }

  public RelativeEncoder getRightEncoder() {
    return encoderRight;
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public static void zeroHeading() {
    navX.isCalibrating();
    navX.reset();
  }

  public AHRS getAhrs() {
    return getAhrs();
  }

  @Override
  public void periodic() {
    m_Odometry.update(navX.getRotation2d(), encoderLeft.getPosition(), encoderRight.getPosition());

    SmartDashboard.putNumber("Left Encoder Value Meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Value Meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro Heading", getHeading());
  }
    // This method will be called once per scheduler run
}

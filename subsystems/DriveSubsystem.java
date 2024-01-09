// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class DriveSubsystem extends SubsystemBase {
  // Creates a new chassis
  private Spark leftMotors, rightMotors;
  private DifferentialDrive drive;

  private Encoder leftEncoder = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
  private Encoder rightEncoder = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final DifferentialDriveOdometry m_odometry;
  private final Field2d m_field = new Field2d();

  public DriveSubsystem() {
    // This is the constructor for the class
    leftMotors = new Spark(1);
    rightMotors = new Spark(0);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    rightMotors.setInverted(false);
    leftMotors.setInverted(true);

    // For every encoder pulse, which is 1/2,000th of a rotation,
    // the distance travelled by the robot is 1/2,000th of the circumference
    // of the wheel, 2*pi*radius. The radius is 3 inches, but we want to use meters.
    leftEncoder.setDistancePerPulse(1.0 / 2000.0 * 2.0 * Math.PI * Units.inchesToMeters(3.0));
    rightEncoder.setDistancePerPulse(1.0 / 2000.0 * 2.0 * Math.PI * Units.inchesToMeters(3.0));

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    SmartDashboard.putData("Field", m_field);
  }

  public double getEncoderAverageDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public double getGyroAngle() {
    return m_gyro.getAngle();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("Left Pulses", leftEncoder.get());
    SmartDashboard.putNumber("Left Rate", leftEncoder.getRate());

    SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
    SmartDashboard.putNumber("Right Pulses", rightEncoder.get());
    SmartDashboard.putNumber("Right Rate", rightEncoder.getRate());

    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());

    m_field.setRobotPose(m_odometry.getPoseMeters());

    m_odometry.update(m_gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public void setMaxOutputLow() {
    drive.setMaxOutput(Constants.kChassisMaxOutputLow);
  }

  public void setMaxOutputHigh() {
    drive.setMaxOutput(Constants.kChassisMaxOutputHigh);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(-leftSpeed, -rightSpeed);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    m_odometry.resetPosition(m_gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }
}
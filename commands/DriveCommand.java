
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  private final DriveSubsystem m_chassis;
  private double m_speed = 0.0;
  private double m_distance = 0.0;
  private double m_encoderSetpoint;
  private double m_gyroSetpoint = 0.0;

  public DriveCommand(double speed, double distance, DriveSubsystem chassis) {

    m_chassis = chassis;
    addRequirements(m_chassis);

    m_speed = speed;
    m_distance = distance;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_chassis.arcadeDrive(0.0, 0.0);
    m_encoderSetpoint = m_chassis.getEncoderAverageDistance() + m_distance;
    m_gyroSetpoint = m_chassis.getGyroAngle();

  }

  @Override
  public void execute() {

    double turningValue = (m_gyroSetpoint - m_chassis.getGyroAngle()) * 0.05; // Sometimes this small number is called kP
    m_chassis.arcadeDrive(-m_speed, turningValue);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_chassis.arcadeDrive(0.0, 0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return (m_chassis.getEncoderAverageDistance() >= m_encoderSetpoint);
  }
}
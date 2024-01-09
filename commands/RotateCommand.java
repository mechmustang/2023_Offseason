// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RotateCommand extends CommandBase {

  private final DriveSubsystem m_chassis;
  private double m_rotationSpeed = 0.0;
  private double m_rotationAmount = 0.0;
  private double m_gyroSetpoint = 0.0;
  private double m_rotationDirection = 1.0;
  
  public RotateCommand(double rotationSpeed, double rotationAmount, DriveSubsystem chassis) {

    m_chassis = chassis;
    addRequirements(m_chassis);

    m_rotationSpeed = rotationSpeed;
    m_rotationAmount = rotationAmount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.arcadeDrive(0.0, 0.0);

    m_gyroSetpoint = m_chassis.getGyroAngle() + m_rotationAmount;

    // Should we be rotating to the right or left?
    if (m_gyroSetpoint > m_chassis.getGyroAngle()) {
      m_rotationDirection = 1.0;
    } else {
      m_rotationDirection = -1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.arcadeDrive(0, m_rotationSpeed * m_rotationDirection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_rotationDirection == 1.0) {
      return (m_chassis.getGyroAngle() >= m_gyroSetpoint);
    } else {
      return (m_chassis.getGyroAngle() <= m_gyroSetpoint);
    }
  }
}

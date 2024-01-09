
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autoCommand;
  private RobotContainer m_robotContainer;
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Pose2d pose = m_robotContainer.m_chassis.getPose();
    SmartDashboard.putNumber("Pose x", pose.getX());
    SmartDashboard.putNumber("Pose y", pose.getX());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }

    m_robotContainer.m_chassis.resetOdometry(new Pose2d());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

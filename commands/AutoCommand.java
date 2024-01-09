package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCommand extends SequentialCommandGroup {
  private final DriveSubsystem m_chassis;
  
  //creates auto command
  public AutoCommand(DriveSubsystem chassis) {
    m_chassis = chassis;

    // NOTE! We are currently experimenting with PathPlanner. The PathPlanner auto
    // does NOT use this command at all, and changing it will have no effect. Instead,
    // edit getAutonomousCommand() in RobotContainer.
          
    addCommands(
      new DriveCommand(0.8, 1.0, m_chassis),
      new RotateCommand(0.6, 90, chassis),
      new DriveCommand(0.8, 1.0, m_chassis),
      new RotateCommand(0.6, 90, chassis),
      new DriveCommand(0.8, 1.0, m_chassis),
      new RotateCommand(0.6, 90, chassis),
      new DriveCommand(0.8, 1.0, m_chassis),
      new RotateCommand(0.6, 90, chassis)
    );
  }
}
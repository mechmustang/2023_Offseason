// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  // The robot's subsystems and commands are defined here
  private final Joystick m_XboxController;
  public final DriveSubsystem m_chassis = new DriveSubsystem();
  //public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //private final AutoCommand m_autoCommand = new AutoCommand(m_chassis);
  
  
  //Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    m_XboxController = new Joystick(0);

    configureBindings();

    //m_intakeSubsystem.resetEncoders();

    m_chassis.setDefaultCommand(
      new RunCommand(() -> m_chassis.arcadeDrive(
        m_XboxController.getRawAxis(Constants.kXBox.leftYAxis),
        m_XboxController.getRawAxis(Constants.kXBox.rightXAxis)),
        m_chassis));
        
  }

  private void configureBindings() {
    final JoystickButton slowTrigger = new JoystickButton(m_XboxController, Constants.kXBox.buttonRightBumper);
    //final JoystickButton forwardArmButton = new JoystickButton(m_XboxController, Constants.kXBox.buttonB);
    //final JoystickButton reverseIntakeButton = new JoystickButton(m_XboxController, Constants.kXBox.buttonY);

    slowTrigger.onTrue(new InstantCommand(m_chassis::setMaxOutputLow, m_chassis)).onFalse(new InstantCommand(m_chassis::setMaxOutputHigh, m_chassis));
    //forwardArmButton.onTrue(new InstantCommand(m_intakeSubsystem::forwardArm, m_intakeSubsystem)).onFalse(new InstantCommand(m_intakeSubsystem::homeArm, m_intakeSubsystem));
    //reverseIntakeButton.onTrue(new InstantCommand(m_intakeSubsystem::reverseIntake, m_intakeSubsystem)).onFalse(new InstantCommand(m_intakeSubsystem::stopIntake, m_intakeSubsystem));
  }

  public Command getAutonomousCommand() {
    // return m_autoCommand;

    // This will load the file "SamplePath.path" and generate it with a max velocity and max acceleration
    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("SamplePath", new PathConstraints(4.0, 1.0));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("MarkerCommand", new InstantCommand());

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
      m_chassis::getPose,
      m_chassis::resetOdometry,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new DifferentialDriveKinematics(Units.inchesToMeters(23.0)),
      m_chassis::tankDrive,
      eventMap,
      m_chassis
    );

    Command fullAuto = autoBuilder.fullAuto(pathGroup);
    return fullAuto;
  }

}
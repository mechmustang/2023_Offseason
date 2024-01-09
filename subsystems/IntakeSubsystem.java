// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase {

  private SparkMaxPIDController m_leftArmPidController;
  private CANSparkMax m_leftArmMotor;
  private final RelativeEncoder m_leftArmEncoder;

  private SparkMaxPIDController m_rightArmPidController;
  private CANSparkMax m_rightArmMotor;
  private final RelativeEncoder m_rightArmEncoder;

  private SparkMaxPIDController m_intakePidController;
  private CANSparkMax m_intakeMotor;
  private final RelativeEncoder m_intakeEncoder;

  /** Creates a new armLift. */
  public IntakeSubsystem() {
    m_leftArmMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_rightArmMotor = new CANSparkMax(4, MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(6, MotorType.kBrushless);

    m_leftArmMotor.setInverted(false);
    m_rightArmMotor.setInverted(true);
    m_intakeMotor.setInverted(false);



    m_leftArmPidController = m_leftArmMotor.getPIDController();
    m_rightArmPidController = m_rightArmMotor.getPIDController();
    m_intakePidController = m_intakeMotor.getPIDController();

    m_leftArmEncoder = m_leftArmMotor.getEncoder();
    m_rightArmEncoder = m_leftArmMotor.getEncoder();
    m_intakeEncoder = m_intakeMotor.getEncoder();

    m_leftArmMotor.setSmartCurrentLimit(30, 20, 0);
    m_rightArmMotor.setSmartCurrentLimit(30, 20, 0);
    m_intakeMotor.setSmartCurrentLimit(30, 20, 0);

    // Left Arm Pid Constants
    m_leftArmPidController.setP(Constants.kArm.kArmAngleP);
    m_leftArmPidController.setI(Constants.kArm.kArmAngleI);
    m_leftArmPidController.setD(Constants.kArm.kArmAngleD);
    m_leftArmPidController.setIZone(Constants.kArm.kArmAngleIz);
    m_leftArmPidController.setFF(Constants.kArm.kArmAngleFF);
    m_leftArmPidController.setOutputRange(Constants.kArm.kArmAngleMinOutput, Constants.kArm.kArmAngleMaxOutput);

    // Right Arm Pid Constants
    m_rightArmPidController.setP(Constants.kArm.kArmAngleP);
    m_rightArmPidController.setI(Constants.kArm.kArmAngleI);
    m_rightArmPidController.setD(Constants.kArm.kArmAngleD);
    m_rightArmPidController.setIZone(Constants.kArm.kArmAngleIz);
    m_rightArmPidController.setFF(Constants.kArm.kArmAngleFF);
    m_rightArmPidController.setOutputRange(Constants.kArm.kArmAngleMinOutput, Constants.kArm.kArmAngleMaxOutput);

    // Intake Pid Constants
    m_intakePidController.setP(Constants.kIntake.kIntakeAngleP);
    m_intakePidController.setI(Constants.kIntake.kIntakeAngleI);
    m_intakePidController.setD(Constants.kIntake.kIntakeAngleD);
    m_intakePidController.setIZone(Constants.kIntake.kIntakeAngleIz);
    m_intakePidController.setFF(Constants.kIntake.kIntakeAngleFF);
    m_intakePidController.setOutputRange(Constants.kIntake.kIntakeAngleMinOutput, Constants.kIntake.kIntakeAngleMaxOutput);

    m_leftArmMotor.burnFlash();
    m_rightArmMotor.burnFlash();
    m_intakeMotor.burnFlash();

  }

  public void resetEncoders() {
    System.out.println("Reset Encoders");
    m_intakeEncoder.setPosition(0);
    m_leftArmEncoder.setPosition(0);
    m_rightArmEncoder.setPosition(0);

  }

  public void homeArm() {
    System.out.println("Move Arm To Home Position And Stop Intake");
    m_leftArmPidController.setReference(Constants.kArm.kArmHomePos, CANSparkMax.ControlType.kSmartMotion);
    m_rightArmPidController.setReference(Constants.kArm.kArmHomePos, CANSparkMax.ControlType.kSmartMotion);
    m_intakeMotor.stopMotor();
  }

  public void forwardArm() {
    System.out.println("Move Arm To Forward Position And Intake Forward");
    m_leftArmPidController.setReference(Constants.kArm.kArmForwardPos, CANSparkMax.ControlType.kSmartMotion);
    m_rightArmPidController.setReference(Constants.kArm.kArmForwardPos, CANSparkMax.ControlType.kSmartMotion);
    m_intakePidController.setReference(Constants.kIntake.kMotorSpeed, CANSparkMax.ControlType.kVoltage);
  }

  public void reverseIntake() {
    System.out.println("Spin Intake Backward");
    m_intakePidController.setReference(-Constants.kIntake.kMotorSpeed, CANSparkMax.ControlType.kVoltage);
  }

  public void stopIntake() {
    System.out.println("Stop Intake");
    m_intakeMotor.stopMotor();
    m_leftArmMotor.stopMotor();
    m_rightArmMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Encoder", m_intakeEncoder.getPosition());
    SmartDashboard.putNumber("Right Arm Encoder", m_rightArmEncoder.getPosition());
    SmartDashboard.putNumber("Left Arm Encoder", m_leftArmEncoder.getPosition());
  }

}

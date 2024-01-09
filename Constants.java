// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static final double kChassisMaxOutputLow = 0.4;
  public static final double kChassisMaxOutputHigh = 0.8;

  public final class kXBox {
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1;
    public static final int rightXAxis = 4;
    public static final int rightYAxis = 5;
    public static final int buttonRightBumper = 6;

    public static final int buttonA = 1;
    public static final int buttonB = 2;
    public static final int buttonX = 3;
    public static final int buttonY = 4;
  }

  public final class kArm {

    public static final double kArmHomePos = 0.0;
    public static final double kArmForwardPos = 120.0;

    public static final double kMotorSpeed = 1.0;

    public static final double kArmAngleSpeed = 0.50;
    public static final double kArmAngleP = 0.00004;    // Make changes here
    public static final double kArmAngleI = 0;
    public static final double kArmAngleD = 0;
    public static final double kArmAngleIz = 0;
    public static final double kArmAngleFF = 0.00014;    // and here
    public static final double kArmAngleMaxOutput = 1; 
    public static final double kArmAngleMinOutput = -1;
  }

  public final class kIntake {

    public static final double kIntakePos = 0;
    public static final double kMotorSpeed = 2.0;

    public static final double kIntakeAngleSpeed = 0.50;
    public static final double kIntakeAngleP = 0.00004;    // Make changes here
    public static final double kIntakeAngleI = 0;
    public static final double kIntakeAngleD = 0;
    public static final double kIntakeAngleIz = 0;
    public static final double kIntakeAngleFF = 0.0014;    // and here
    public static final double kIntakeAngleMaxOutput = 1; 
    public static final double kIntakeAngleMinOutput = -1;
  }

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
}
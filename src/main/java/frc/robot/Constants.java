// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import swervelib.math.Matter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
/* 
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

*/

// coral intake wrong direction
// elevator wrong direction
// swerve = uncalibrated

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = (4.4 * 0.5);
  // Max limit speed is halved
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class ButtonConstants
  {
    public static final int xboxA = 1;
    public static final int xboxB = 2;
    public static final int xboxX = 3;
    public static final int xboxY = 4;

    public static final int xboxLB = 5;
    public static final int xboxRB = 6;
    public static final int xboxMAPS = 7;
    public static final int xboxLINES = 8;
    public static final int xboxLeftJoystickDown = 9;
    public static final int xboxRightJoystickDown = 10;
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ElevatorConstants {
    public static final int elevatorUpperMotorID = 9;
    public static final int elevatorLowerMotorID = 10;
    // Motor ID 10 is the bad one, so lower ID is bad one and also must be positive speed

    // public static final double kGearRatio = 4.4 * 300;
    // Calculated by Lok
    public static final double kGearRatio = 0.255;
    
    // up to level 2 for now
    public static final int kMaxLevel = 2;
    public static final int kMinLevel = 0;
    //public static final double kIntakeBaseHeight = 28.55;
    public static final double vL1Height = 0.0;
    // set base vertical height
    public static final double eL1Height = 0.0;
    // set base slant height

    // TODO: 28.55 IS UNCERTAIN. MEASURE INTAKE HEIGHT FROM GROUND.

    public static final double vL2Height = 3.17;
    // 31.72 - 28.55
    public static final double eL2Height = 3.22;
    // 3.17 / sin(79.99)

    public static final double vL3Height = 19.04;
    // 47.59 - 28.55
    public static final double eL3Height = 19.33;
    // 19.04 / sin(79.99)

    public static final double vL4Height = 43.32;
    // 71.87 - 28.55
    public static final double eL4Height = 43.98;
    // 43.32 / sin(79.99)

    public static final double kElevatorDefaultTolerance = 1.0;

    public static final double kPositionConversionFactor = 1.0; // Adjust based on encoder specs
    public static final double kZeroOffset = 0.0; // Adjust if needed

    public static final double kEncoderCountsPerRotation = 42;
    public static final double kMaxSpeedPercentage = 0.15;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static final class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class IntakeConstants
    {

      public static final double AlgaeIntakeSpeeds  = 0.1;
      public static final double AlgaeOuttakeSpeeds = -0.1;
      public static       int    algaeUpperMotorID  = 21;

      public static final double CoralIntakeSpeeds  = -0.1;
      public static final double CoralOuttakeSpeeds = -0.3;
      public static final int    coralMotorID  = 11;
      public static final double kIntakeReduction    = 0;
    }

  public static final class LimelightConstants
  {
    
  }
}

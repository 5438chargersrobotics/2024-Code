// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

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

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 0.75;
  }
  public static final class ShooterConstants {
    public static final boolean kEncoderReversed = false;
    public static final int kEncoderCPR = 1;
    public static final double kEncoderDistancePerPulse =
        // Distance units will be rotations
        1.0 / (double) kEncoderCPR;

    public static final int kShooterMotorPort = 10;
    public static final int kShooterMotor1Port = 12;

    public static final double kShooterFreeRPS = 5676;
    public static final double kShooterTargetRPS = 4000;
    public static final double kShooterToleranceRPS = 50;

    // These are not real PID gains, and will have to be tuned for your specific robot.
    public static final double kP = 0.001;
    public static final double kI = 0;
    public static final double kD = 0;

    // On a real robot the feedforward constants should be empirically determined; these are
    // reasonable guesses.
    public static final double kSVolts = 0.05;
    public static final double kVVoltSecondsPerRotation =
        // Should have value 12V at free speed...
        12.0 / kShooterFreeRPS;
  }

  public static final class IntakeConstants{
    public static final int kIntakeMotorPort = 13;
    public static final double kIntakeMotorSpeed = -1;
    public static final int kLeftDistanceSensorPort = 1;
     public static final int kRightDistanceSensorPort = 2;
  }

  public static final class  ClimbConstants {
    public static final int kleftClimberMotor = 14;
    public static final int krightClimberMotor = 15;
    public static final double kMiddleClimbSpot = 20;
    public static final double kEdgeClimbSpot = 30;
    public static final double kP = 0.1;
    
  }

  public static final class ArmConstants {
    public static final int kMotorPort = 11;
    public static final int kMotor1Port = 9;

    public static final double kP = 1.3;

    // These are fake gains; in actuality these must be determined individually for each robot
    // public static final double kSVolts = 0.18;
    // public static final double kGVolts = 0.3;
    // public static final double kVVoltSecondPerRad = 6.24;
    // public static final double kAVoltSecondSquaredPerRad = 0.07;

    public static final double kMaxVelocityRadPerSecond = 1;
    public static final double kMaxAccelerationRadPerSecSquared = 2;

    // public static final int[] kEncoderPorts = new int[] {4, 5};
    // public static final int kEncoderPPR = 2048;
    // public static final double m_encoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;
    public static final double m_EncoderDistancePerRotation = 2.0 * Math.PI;
    public static final int kEncoderPort = 0;
    public static final double kEncoderDifference = 0;
    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = 2.681;//3.715;
    public static final double kAmpSpot = 1.0;//2.035;
    public static final double kSubwooferSpot = 2.375;//2.406
    public static final double kSourceSpot = 1.61; 
    public static final double kStowSpot = 2.556;//3.6;
    public static final double kPodiumSpot = 1.9;//3.02;
    public static final double kWingLineSpot = 1.85;//2.94;
   public static final double kMiddleSpot = 2.05;//3.181;
   public static final double kSubwooferSideSpot = 2.529;//3.573;
   public static final double kHoardSpot = 1.52;//2.611;
   public static final double kMiddleAutoSpot  = 2.096;//3.14;
   public static final double kPodiumAutoSpot = 1.98;
  
  }

  public static final class AutoConstants {
    public static final double kAutoTimeoutSeconds = 12;
    public static final double kAutoShootTimeSeconds = 7;
  }

  public static final class OIConstants {
    public static final int kOperatorControllerPort = 1;
  }

  public static final class Field{
    public static final Translation2d blueSpeaker = new Translation2d(Units.inchesToMeters(8.5), Units.inchesToMeters(214));//223.42
    public static final Translation2d redSpeaker = new Translation2d(Units.inchesToMeters(642.73), Units.inchesToMeters(214));//213.42
    public static final double noteVelocity = 10;//642.73
    public static final Translation2d blueHoardSpot = new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(260));//272
     public static final Translation2d redHoardSpot = new Translation2d(Units.inchesToMeters(638), Units.inchesToMeters(260));
  }
}


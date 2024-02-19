// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_Arm = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_Arm1 = new CANSparkMax(ArmConstants.kMotor1Port, MotorType.kBrushless);
  // private final Encoder m_encoder =
  //     new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]);
  private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(ArmConstants.kEncoderPort);
  private final PIDController pidController = new PIDController(ArmConstants.kP, 0, 0);
  private final TrapezoidProfile.Constraints trapezoidProfile = new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared);
   private final ProfiledPIDController pidController1 = new ProfiledPIDController(0.5, 0, 0,trapezoidProfile);
   NetworkTableInstance inst = NetworkTableInstance.getDefault();
   private NetworkTableEntry ty = inst.getTable("limelight").getEntry("ty");
  
  // private final ArmFeedforward m_feedforward =
  //     new ArmFeedforward(
  //         ArmConstants.kSVolts, ArmConstants.kGVolts,
  //         ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    m_Encoder.setDistancePerRotation(ArmConstants.m_EncoderDistancePerRotation);
    m_Arm.follow(m_Arm1, true);
   // pidController1.reset(getMeasurement());
  
  }

  // @Override
  // public void useOutput(double output, TrapezoidProfile.State setpoint) {
  //   // Calculate the feedforward from the sepoint
  //   //double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
  //   // Add the feedforward to the PID output to get the motor output
  //   m_Arm1.setVoltage(-output ); 
   
  // }
  @Override
  public void periodic(){
    SmartDashboard.putNumber("ArmEncoder", getMeasurement());
   
  }

 // @Override
  public double getMeasurement() {
    return m_Encoder.getDistance();
  }

 public void setMotor(double setpoint){
  m_Arm1.set(pidController.calculate(m_Encoder.getDistance(), setpoint));
 }
 public void setMotorSpeed(double speed){
  m_Arm1.set(speed);
 }

 public void stopMotor(){
  m_Arm1.set(0);
 }

 public double calculateArmAngle(){
  double targetOffsetAngleVertical = ty.getDouble(0);
  double shotAngle = 0.0001 * Math.pow(targetOffsetAngleVertical,2.0) + .002*targetOffsetAngleVertical+.4992;
  return shotAngle;
 }
}

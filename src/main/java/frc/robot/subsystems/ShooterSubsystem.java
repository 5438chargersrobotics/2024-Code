// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ShooterSubsystem extends PIDSubsystem {
  private final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor1 = new CANSparkMax(ShooterConstants.kShooterMotor1Port, MotorType.kBrushless);
 private final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  /** The shooter subsystem for the robot. */
  public ShooterSubsystem() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooterEncoder.setPositionConversionFactor(ShooterConstants.kEncoderDistancePerPulse);
    setSetpoint(ShooterConstants.kShooterTargetRPS);
    m_shooterMotor1.follow(m_shooterMotor);
  }
  @Override
  public void periodic(){
   
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_shooterEncoder.getVelocity();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setMotorQuarterSpeed(){
    m_shooterMotor.set(0.25);
  }
  public void setMotorHoardSpeed(){
    m_shooterMotor.set(0.32
    );
  }
  public void setMotorHalfSpeed(){
    m_shooterMotor.set(1);
  }
  public void setMotorTrapSpeed(){
    m_shooterMotor.set(0.28);
  }
  public void setMotorThreeQuarterSpeed(){
    m_shooterMotor.set(0.75);
  }
  public void setMotorFullSpeed(){
    m_shooterMotor.set(1);
  }
  public void setMotorReverse(){
    m_shooterMotor.set(-.1);
  }
  public void stopMotor(){
    m_shooterMotor.set(0);
  }

}

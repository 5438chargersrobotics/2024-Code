package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftClimberMotor = new CANSparkMax(ClimbConstants.kleftClimberMotor, MotorType.kBrushless );
    private final CANSparkMax m_rightClimberMotor = new CANSparkMax(ClimbConstants.krightClimberMotor, MotorType.kBrushless);
    private final PIDController pidController = new PIDController(ClimbConstants.kP, 0, 0);
    private final RelativeEncoder m_rightRelativeEncoder = m_rightClimberMotor.getEncoder();
    private final RelativeEncoder m_leftRelativeEncoder = m_leftClimberMotor.getEncoder();
    private final Pigeon2 m_Pigeon2;
    private final SparkLimitSwitch m_LeftLimitSwitch = m_leftClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    private final SparkLimitSwitch m_RightLimitSwitch = m_leftClimberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

public ClimberSubsystem(){
    m_leftClimberMotor.setIdleMode(IdleMode.kBrake);
    m_rightClimberMotor.setIdleMode(IdleMode.kBrake);
    m_leftRelativeEncoder.setPosition(0);
    m_rightRelativeEncoder.setPosition(0);
    m_Pigeon2 = new Pigeon2(20);
}
public void periodic(){
    SmartDashboard.putNumber("Left Climb Encoder", getLeftClimbEncoderMeasurement());
    SmartDashboard.putNumber("Right Climb Encoder", getRightClimbEncoderMeasurement());
    if(m_LeftLimitSwitch.isPressed()){
        m_leftRelativeEncoder.setPosition(0);
    }
    if(m_RightLimitSwitch.isPressed()){
        m_rightRelativeEncoder.setPosition(0);
    }
}
public void climbWithGyro(){
    m_leftClimberMotor.set(pidController.calculate(m_leftRelativeEncoder.getPosition(), 1-0.1 * m_Pigeon2.getRoll().getValueAsDouble()));
    m_rightClimberMotor.set(pidController.calculate(m_leftRelativeEncoder.getPosition(), 1+0.1 * m_Pigeon2.getRoll().getValueAsDouble()));
}
public double getLeftClimbEncoderMeasurement(){
    return m_leftRelativeEncoder.getPosition();
}
public double getRightClimbEncoderMeasurement(){
    return m_rightRelativeEncoder.getPosition();
}
public void setLeftMotor(double setpoint){
    m_leftClimberMotor.set(pidController.calculate(m_leftRelativeEncoder.getPosition(), setpoint));
   }
public void setRightMotor(double setpoint){
    m_leftClimberMotor.set(pidController.calculate(m_rightRelativeEncoder.getPosition(), setpoint));
   }
public void setBothMotors(double setpoint){
     m_leftClimberMotor.set(pidController.calculate(m_leftRelativeEncoder.getPosition(), setpoint));
      m_leftClimberMotor.set(pidController.calculate(m_rightRelativeEncoder.getPosition(), setpoint));
}

public void setLeftMotorUp(){
    m_leftClimberMotor.set(1);
}
public void setRightMotorUp(){
    m_rightClimberMotor.set(1);
}
public void setLeftMotorDown(){
    m_leftClimberMotor.set(-1);
}
public void setRightMotorDown(){
    m_rightClimberMotor.set(-1);
}
public void setBothMotorsUp(){
    m_rightClimberMotor.set(1);
    m_leftClimberMotor.set(1);
}
public void setBothMotorsDown(){
         m_rightClimberMotor.set(-1);
    m_leftClimberMotor.set(-1);
}

public void stopLeftMotor(){
    m_leftClimberMotor.set(0);
}
public void stopRightMotor(){
    m_rightClimberMotor.set(0);
}
public void stopBothMotors(){
     m_rightClimberMotor.set(0);
    m_leftClimberMotor.set(0);
}
}

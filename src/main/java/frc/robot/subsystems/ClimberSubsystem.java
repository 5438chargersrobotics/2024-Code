package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftClimberMotor = new CANSparkMax(14, MotorType.kBrushless );
    private final CANSparkMax m_rightClimberMotor = new CANSparkMax(15, MotorType.kBrushless);

public ClimberSubsystem(){
    m_leftClimberMotor.setIdleMode(IdleMode.kBrake);
    m_rightClimberMotor.setIdleMode(IdleMode.kBrake);
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

public void stopLeftMotor(){
    m_leftClimberMotor.set(0);
}
public void stopRightMotor(){
    m_rightClimberMotor.set(0);
}
}

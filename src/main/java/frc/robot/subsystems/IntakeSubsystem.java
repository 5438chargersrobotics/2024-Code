package frc.robot.subsystems;

import java.util.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private final CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    private final DigitalInput m_LeftDistanceSensor = new DigitalInput(IntakeConstants.kLeftDistanceSensorPort);
    private final DigitalInput m_RightDistanceSensor = new DigitalInput(IntakeConstants.kRightDistanceSensorPort);
    

    public IntakeSubsystem(){
        m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void runIntake(){
        m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
    }

    public void runIntakeWithSensor(){
     if(!m_LeftDistanceSensor.get() || !m_RightDistanceSensor.get()){
        m_IntakeMotor.set(0);
     }
     else{
        m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
     }
    }
    public boolean getBothSensors(){
        return !m_LeftDistanceSensor.get()&&!m_RightDistanceSensor.get();
    }
    public boolean getLeftSensor(){
        return !m_LeftDistanceSensor.get();
    }
    public boolean getRightSensor(){
        return !m_RightDistanceSensor.get();
    }

    public void runOuttake(){
        m_IntakeMotor.set(-IntakeConstants.kIntakeMotorSpeed);
    }

    public void stopIntake(){
        m_IntakeMotor.set(0);
    }

    }
    

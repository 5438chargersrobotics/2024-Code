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
    private final DigitalInput m_DistanceSensor = new DigitalInput(IntakeConstants.kDistanceSensorPort);
    private final Timer m_Timer = new Timer();
    

    public IntakeSubsystem(){
        m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void runIntake(){
        m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
    }

    public void runIntakeWithSensor(){
        //m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
     if(!m_DistanceSensor.get()){
        m_IntakeMotor.set(0);
     }
     else{
        m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
     }
    }
    public boolean getSensor(){
        return !m_DistanceSensor.get();
    }

    public void runOuttake(){
        m_IntakeMotor.set(-IntakeConstants.kIntakeMotorSpeed);
    }

    public void stopIntake(){
        m_IntakeMotor.set(0);
    }

    }
    

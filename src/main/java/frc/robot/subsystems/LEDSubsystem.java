package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED m_addressableLED = new AddressableLED(2);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(18);
    private IntakeSubsystem m_IntakeSubsystem;
public LEDSubsystem(IntakeSubsystem m_IntakeSubsystem){
    this.m_IntakeSubsystem= m_IntakeSubsystem;
    m_addressableLED.setLength(m_ledBuffer.getLength());
    m_addressableLED.setData(m_ledBuffer);
    m_addressableLED.start();
}
public void setLEDColor() {
    if(m_IntakeSubsystem.getSensor()){
         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, Color.kGreen);
    }
    }

  }
}

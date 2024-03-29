package frc.robot.subsystems;

import java.util.Timer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED m_addressableLED = new AddressableLED(3);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(70);
    private final PS5Controller m_Controller = new PS5Controller(0);
    private final PS4Controller m_operatorController = new PS4Controller(1);
    private final Timer m_Timer = new Timer();
    private IntakeSubsystem m_IntakeSubsystem;
public LEDSubsystem(IntakeSubsystem m_IntakeSubsystem){
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    m_addressableLED.setLength(m_ledBuffer.getLength());
    m_addressableLED.setData(m_ledBuffer);
    m_addressableLED.start();
}
public void setLEDColorGreen() {
    if(m_IntakeSubsystem.getBothSensors()){
         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, Color.kGreen);
      //m_operatorController.setRumble(RumbleType.kLeftRumble, 1);
    }
    m_addressableLED.setData(m_ledBuffer);
    }
    else{
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
     m_addressableLED.setData(m_ledBuffer);
  }
    }
public void setLEDColorYellow() {
  if(m_IntakeSubsystem.getLeftSensor()||m_IntakeSubsystem.getRightSensor()){
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, Color.kGreen);
    }
    m_addressableLED.setData(m_ledBuffer);
   // m_operatorController.setRumble(RumbleType.kBothRumble, 1);
  }
   else{
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
     m_addressableLED.setData(m_ledBuffer);
  }
}
public void setLEDColorRed(){
  for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 100, 100, 55);
    }
    m_addressableLED.setData(m_ledBuffer);
}
public void turnOffLEDs(){
  for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_addressableLED.stop();
    }
}

}

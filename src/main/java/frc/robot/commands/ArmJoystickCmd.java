package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmJoystickCmd extends Command {
    private final ArmSubsystem armSubsystem;
     private final Supplier<Double> speedFunction;

     public ArmJoystickCmd(ArmSubsystem armSubsystem, Supplier<Double> speedFunction){
         this.armSubsystem = armSubsystem;
    this.speedFunction = speedFunction;
         addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double realSpeed = speedFunction.get();
        double motorSpeed = realSpeed;
        armSubsystem.setMotorSpeed(motorSpeed);
    }

    @Override
    public void end(boolean interrupted){
        armSubsystem.setMotorSpeed(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}

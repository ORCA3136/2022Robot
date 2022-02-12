package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import frc.robot.Constants;


public class Conveyor extends SubsystemBase{
    private CANSparkMax conveyor;



public Conveyor(){
    conveyor = new CANSparkMax(Constants.kConveyor1, MotorType.kBrushless);
}

public void lowerConveyor(double ConveyorSpeed){
    conveyor.set(ConveyorSpeed);
}

public void raiseConveyor(double ConveyorSpeed){
    conveyor.set(ConveyorSpeed);
}

public void stopConveyor(){
    conveyor.set(0);
}



}

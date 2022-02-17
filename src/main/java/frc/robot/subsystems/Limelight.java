package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase 
{
    private NetworkTable limelightTable;
    private NetworkTableEntry pipeline;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    
    //configure pipelines here
    private static int LED_OFF = 1;
    private static int LED_ON= 2;
    private static int RING = 0;

    public Limelight()
    {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        pipeline = limelightTable.getEntry("pipeline");
        
        

    }

    public void enableLED()
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void disableLED()
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void findRing()
    {
        pipeline.setNumber(RING);
    }
    
    @Override   
    public void periodic(){
        //TODO - put data out to the smartdashboard
        SmartDashboard.putNumber("PIPELINE", pipeline.getDouble(0));


    }
    
}

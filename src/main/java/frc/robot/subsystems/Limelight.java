package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.models.OnTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.AimAndShoot;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.FlyWheel;

public class Limelight extends SubsystemBase 
{
    private NetworkTable limelightTable;
    private NetworkTableEntry pipeline;
    private NetworkTableEntry ledMode;
    private boolean alignedToTarget = false;
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private double Kp = -0.1; 
    private boolean done = false;

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




    public OnTarget aim() {
      //read values periodically
      table = NetworkTableInstance.getDefault().getTable("limelight");
      tx = table.getEntry("tx");
      double x = tx.getDouble(0.0);
  
      OnTarget target = new OnTarget(); 


      double steeringAdjust = Kp * x;
      double limelightFlywheel = Kp *x; //Not sure what this is for? or why?
      double limelightConveyor = Kp *x; //not sure what this is for or why?

      double left=steeringAdjust;
      double right=-steeringAdjust;
      double flyWheelSpeed = limelightFlywheel;
      double conveyorSpeed = limelightConveyor;


      target.setLeftPower(left);
      target.setRightPower(right);
      target.setBigAngle(x);
      target.setFlyWheelLL(flyWheelSpeed);
      target.setConveyorLL(conveyorSpeed);

      return target;

    }
    
    

}

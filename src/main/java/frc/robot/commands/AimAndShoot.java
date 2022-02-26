package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel;
import frc.robot.Constants;

public class AimAndShoot extends CommandBase{
    private Drivetrain m_drivetrain;
    private boolean alignedToTarget = false;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private FlyWheel m_FlyWheel;
    private Conveyor m_Conveyor;
    private NetworkTableEntry tv;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 45;
    
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 22.125;
    
    // distance from the target to the floor
    double goalHeightInches = 103;
    
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    private double limelightHeightInches;
    
    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);

    private double Kp = -0.1;  // Proportional control constant

    private boolean done = false;

  
   
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     * @return 
     */
    public AimAndShoot(Drivetrain driveSubsystem, FlyWheel flyWheelSubsystem, Conveyor conveyorSubsystem) {
      m_drivetrain = driveSubsystem;
      m_FlyWheel = flyWheelSubsystem;
      m_Conveyor = conveyorSubsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(driveSubsystem);
      addRequirements(flyWheelSubsystem);
      addRequirements(conveyorSubsystem);     
    }    
    
    // Called when the command is initially scheduled.
    public void initialize() 
    {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      //LIMELIGHT
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
       //read values periodically
       table = NetworkTableInstance.getDefault().getTable("limelight");
       tx = table.getEntry("tx");
       ty = table.getEntry("ty");
       ta = table.getEntry("ta");
       tv = table.getEntry("tv");
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double v = tv.getDouble(0.0);
  
       double steeringAdjust = Kp * x;
       
      if (v == 0.0f)
      {
           // We don't see the target, seek for the target by spinning in place at a safe speed.
           steeringAdjust = 0.3f;
      }
      else
      {
           // We do see the target, execute aiming code
           steeringAdjust = Kp * x;
      }

      if(Math.abs(x)<.2)
      {
        done=true;
      }
      else
      {
        done = false;
      }

      double limeLightFlywheel = Kp * x;
      
      if(Math.abs(x)<.2)
      {
          done=true;
      }
      else
      {
        done=false;
      }

      double limeLightConveyor = Kp * x;

      if(Math.abs(x)<.2)
      {
        done = true;
      }
      else
      {
        done = false;
      }

      double left=steeringAdjust;
      double right=-steeringAdjust;
      double flyWheelSpeed=limeLightFlywheel;
      double conveyorSpeed=limeLightConveyor;
     
      m_drivetrain.drivePercent(left, right);
      m_FlyWheel.shoot(flyWheelSpeed);
      m_Conveyor.raiseConveyor(conveyorSpeed);
    }
  
    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.



    public boolean isFinished() {
      return done;
    }

}

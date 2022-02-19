package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel;
import frc.robot.Constants;

public class AimAndShoot {
    private Drivetrain m_drivetrain;
    private boolean alignedToTarget = false;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private FlyWheel m_FlyWheel;
    private Conveyor m_Conveyor;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 90;
    
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;
    
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
    public void AimAndShoot(Drivetrain subsystem) {
      m_drivetrain = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);     
    }    
    private void addRequirements(Drivetrain subsystem) {
    }
    public void AimAndShoot(FlyWheel subsystem) {
        m_FlyWheel = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }

    public void AimAndShoot(Conveyor subsystem){
        m_Conveyor = subsystem;
        addRequirements(subsystem);
    }
  
    private void addRequirements(Conveyor subsystem) {
    }
    private void addRequirements(FlyWheel subsystem) {
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
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
  
       double steeringAdjust = Kp * x;
      if(Math.abs(x)<.2)
      {
        done=true;
      }
      else
      {
        done = false;
      }

      double LimeLightFlywheel = Kp * x;
      if(Math.abs(x)<.2)
      {
          done=true;
      }
      else
      {
        done=false;
      }

      double LimeLightConveyor = Kp * x;
      if(Math.abs(x)<.2)
      {
          done=true;
      }
      else
      {
        done=false;
      }

      double left=steeringAdjust;
      double right=-steeringAdjust;
      double FlyWheelSpeed=LimeLightFlywheel;
      double ConveyorSpeed=LimeLightConveyor;
     
      m_drivetrain.drivePercent(left, right);
      m_FlyWheel.shoot(FlyWheelSpeed);
      m_Conveyor.raiseConveyor(ConveyorSpeed);
    }
  
    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.



    public boolean isFinished() {
      return done;
    }

}

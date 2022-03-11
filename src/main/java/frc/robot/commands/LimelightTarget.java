package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightTarget extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;
    private final FlyWheel flywheel;
    private final Conveyor conveyor;
    private final Intake intake;

    private boolean alignedToTarget = false;
    private boolean targetInRange = false;
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    private double Kp = -0.1; // Proportional control constant

    private boolean done = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public LimelightTarget(Drivetrain dt, FlyWheel fw, Conveyor con, Intake in) {
        drivetrain = dt;
        flywheel = fw;
        conveyor = con;
        intake = in;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(dt);
        addRequirements(fw);
        addRequirements(con);
        addRequirements(in);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        // LIMELIGHT

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // read values periodically
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);

        double steeringAdjust = Kp * x;
        double distanceAdjust = Kp * y;
        //align to target first //might need to add a check for in not aligned to target
        if (Math.abs(x) < .2) 
        {
            alignedToTarget = true;
        } else {
            alignedToTarget = false;
            double left = steeringAdjust;
            double right = -steeringAdjust;
            //move to position
            drivetrain.drivePercent(left, right);
        }
        //now move to right distance
        if(alignedToTarget)
        {
            if (Math.abs(y) < .2) 
            {
                targetInRange = true;
            } else {
                targetInRange = false;  
                //move to position
                drivetrain.drivePercent(distanceAdjust, distanceAdjust);
            }
        }

        if(alignedToTarget && targetInRange)
        {
            //TODO add some logic to start flywheel
            flywheel.PIDshoot(Constants.kShooterHighTargetRPS, Constants.kShooterHighTargetF3RPS);
            //add a delay for ramp.. or have this method provide a response that indicates it is ready,...
            
            //add some logic to start conveyor
            //add some logic to start intake
            done = true;
        }
        else{
            done = false;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //stop flywheel
        //stop intake
        //stop conveyor
        //stop drivetrain?
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }

}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Drivetrain extends SubsystemBase {

    private DifferentialDrive diffDrive;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);


    private SparkMaxPIDController leftController;
    private SparkMaxPIDController rightController;

    private final CANSparkMax leftLeader;
    private final CANSparkMax leftFollower;
    private final CANSparkMax leftFollower2;

    private final CANSparkMax rightLeader;
    private final CANSparkMax rightFollower;
    private final CANSparkMax rightFollower2;

    private final Field2d fieldSim = new Field2d();

    private double lastLeftVelocityMPS = 0.0;
    private double lastRightVelocityMPS = 0.0;
    private double afterEncoderReduction = 6.0; // Internal encoders

    private RelativeEncoder leftInternalEncoder;
    private RelativeEncoder rightInternalEncoder;

    private final SimpleMotorFeedforward leftModel, rightModel;

    private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
       
    private double baseDistanceLeftRad = 0.0;
    private double baseDistanceRightRad = 0.0;

    //LIMELIGHT STUFF
    private NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;



    public Drivetrain() 
    {
        //LIMELIGHT
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        //TODO SET THESE BASD ON CHARACTERIZATION?
        leftModel = new SimpleMotorFeedforward(0.20554, 0.10965, 0.016329);
        rightModel = new SimpleMotorFeedforward(0.20231, 0.11768, 0.0085871);

        leftLeader     = new CANSparkMax(Constants.LEADER_LEFT_CAN_ID, MotorType.kBrushless);
        leftFollower   = new CANSparkMax(Constants.FOLLOWER_LEFT_CAN_ID, MotorType.kBrushless);
        leftFollower2  = new CANSparkMax(Constants.FOLLOWER_LEFT_2_CAN_ID, MotorType.kBrushless);

        rightLeader    = new CANSparkMax(Constants.LEADER_RIGHT_CAN_ID, MotorType.kBrushless);
        rightFollower  = new CANSparkMax(Constants.FOLLOWER_RIGHT_CAN_ID, MotorType.kBrushless);
        rightFollower2 = new CANSparkMax(Constants.FOLLOWER_RIGHT_2_CAN_ID, MotorType.kBrushless);

        leftInternalEncoder = leftLeader.getEncoder();
        rightInternalEncoder = rightLeader.getEncoder();
        leftInternalEncoder.setPosition(0.0);
        rightInternalEncoder.setPosition(0.0);

        if (Constants.RESET_SPARKMAX) {
            leftLeader.restoreFactoryDefaults();
            leftFollower.restoreFactoryDefaults();
            rightLeader.restoreFactoryDefaults();
            rightFollower.restoreFactoryDefaults();
            leftFollower2.restoreFactoryDefaults();
            rightFollower2.restoreFactoryDefaults();  
          }

        //set the the leader / followers...
        leftFollower.follow(leftLeader);
        leftFollower2.follow(leftLeader);
        rightFollower.follow(rightLeader);
        rightFollower2.follow(rightLeader);
        
        //Invert the right side.
        leftLeader.setInverted(true);
    
        //Do some more specific configruations on the Maxes
        leftLeader.enableVoltageCompensation(12.0);
        rightLeader.enableVoltageCompensation(12.0);
        //set current limit - need to research why, assume this is like capping the speed at 80% of values
        leftLeader.setSmartCurrentLimit(80);
        leftFollower.setSmartCurrentLimit(80);
        rightLeader.setSmartCurrentLimit(80);
        rightFollower.setSmartCurrentLimit(80);
        leftFollower2.setSmartCurrentLimit(80);
        rightFollower2.setSmartCurrentLimit(80);
        //set the can timeout - doing this apparently makes the calls non-blocking and means errors are checked in a separate thread through the driverstation
        leftLeader.setCANTimeout(0);
        leftFollower.setCANTimeout(0);
        leftFollower2.setCANTimeout(0);
        rightLeader.setCANTimeout(0);
        rightFollower.setCANTimeout(0);
        rightFollower2.setCANTimeout(0);
        
        //set everything inthe flash
        if (Constants.RESET_SPARKMAX) {
            leftLeader.burnFlash();
            leftFollower.burnFlash();
            rightLeader.burnFlash();
            rightFollower.burnFlash();
            leftFollower2.burnFlash();
            rightFollower2.burnFlash();  
          }

        //suspect that it is because there was a leader/follower relationship 
        //that is why we had issues with tank drive....
       diffDrive = new DifferentialDrive(leftLeader, rightLeader);
             
        leftEncoder = leftLeader.getEncoder();
        leftController = (SparkMaxPIDController) leftLeader.getPIDController();
        ((SparkMaxPIDController) leftController) . setFeedbackDevice (leftEncoder);

        rightEncoder = rightLeader.getEncoder();
        rightController = (SparkMaxPIDController) rightLeader.getPIDController();
        ((SparkMaxPIDController) rightController) . setFeedbackDevice (rightEncoder);
        
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

    }

    /**
     * A really basic drive controller - this should not really be used going forward as we are trying to use a cleaner method outlined by other
     * more sophisticated teams to use voltages - vs. jsut raw values. 
     * @param controller
     */
    public void drive( XboxController controller) 
    {
     diffDrive.tankDrive(trueLeftX((controller.getLeftY() * Constants.kLeftDriveScaling)), 
     trueRightX((controller.getRightY() * Constants.kRightDriveScaling)*-1));
    }

    /**
     * Should be the key method gonig forward for a basic drive command. takes in the controller so we can capture the 
     * throttle from the joysticks. This handles deadband to make sure we don't get creep when idle as well as capping
     * the max speed to help the team learn and make the robot more controllable.
     * 
     * TODO - I think I can remove the maxSpeed element since the speed should be capped by the MAX_VELOCITY_MPS I jsut had a bad calculation in there for 
     * what that value shoudl be which was above 100% as I think our max speed is like 13 ft / second.
     * @param controller
     * @param maxSpeed
     * @return 
     */
    public void drivePercentController(XboxController controller, double maxSpeed)
    {
        driveVelocity(trueLeftX(((controller.getLeftY() * Constants.MAX_VELOCITY_MPS))), (trueRightX((controller.getRightY() * Constants.MAX_VELOCITY_MPS))));
    }


    /**
     * Same as percentController method but takes in the raw values. from -1 to 1
     * @param left
     * @param right
     */
    public void drivePercent(double left, double right)
    {

        SmartDashboard.putNumber("LEFT", left);
        SmartDashboard.putNumber("RIGHT", right);
        driveVelocity(left*Constants.MAX_VELOCITY_MPS, right*Constants.MAX_VELOCITY_MPS);
        
    }

    /**
     * takes the speed and works to conver it all to volts since volts is what we really need / want for kinematics.
     * TODO - get the output values on shuffleboard so I can understand what is going on!!!!
     * 
     * @param leftVelocityMPS
     * @param rightVelocityMPS
     */
    public void driveVelocity(double leftVelocityMPS, double rightVelocityMPS)
    {

        SmartDashboard.putNumber("LEFT MPS", leftVelocityMPS);
        SmartDashboard.putNumber("RIGHT MPS", rightVelocityMPS);
        SmartDashboard.putNumber("GYRO ANGLE", gyro.getAngle());
        double maxAccelerationPerCycle = Double.POSITIVE_INFINITY * Constants.loopPeriodSecs;
        double leftAcceleration = lastLeftVelocityMPS > 0 
        ? leftVelocityMPS - lastLeftVelocityMPS 
        : lastLeftVelocityMPS - leftVelocityMPS;
        //Shuffleboard.getTab("Drive Details").add("LEFT ACCELERATION", leftAcceleration);

        if(leftAcceleration> maxAccelerationPerCycle)
        {
            lastLeftVelocityMPS += leftVelocityMPS > 0 ? maxAccelerationPerCycle : -maxAccelerationPerCycle;
        }
        else
        {
            lastLeftVelocityMPS = leftVelocityMPS;
        }

        double rightAcceleration = lastRightVelocityMPS > 0 
        ? rightVelocityMPS - lastRightVelocityMPS 
        : lastRightVelocityMPS - rightVelocityMPS;
       // Shuffleboard.getTab("Drive Details").add("RIGHT ACCELERATION", rightAcceleration);

        if(rightAcceleration> maxAccelerationPerCycle)
        {
            lastRightVelocityMPS += rightVelocityMPS > 0 ? maxAccelerationPerCycle : -maxAccelerationPerCycle;
        }
        else
        {
            lastRightVelocityMPS = rightVelocityMPS;
        }

        //calculate the setpoint and the feed forward voltage
        double leftVelocityRPS = lastLeftVelocityMPS / Constants.WHEEL_RADIUS_METERS;
        double rightVelocityRPS = lastRightVelocityMPS / Constants.WHEEL_RADIUS_METERS;
       // Shuffleboard.getTab("Drive Details").add("LEFT VELOCITY RPS", leftVelocityRPS);
      //  Shuffleboard.getTab("Drive Details").add("RIGHT VELOCITY RPS", rightVelocityRPS);

        double leftFFVolts = leftModel.calculate(leftVelocityRPS);
        double rightFFVolts = rightModel.calculate(rightVelocityRPS);

     //   Shuffleboard.getTab("Drive Details").add("LEFT FF Volts", leftFFVolts);
    //    Shuffleboard.getTab("Drive Details").add("RIGHT FF Volts", rightFFVolts);
        SmartDashboard.putNumber("LEFT FF VOLTS", leftFFVolts);
        SmartDashboard.putNumber(" RIGHT FF VOLTS", rightFFVolts);

        //this is just a basic drive -
        //leftLeader.setVoltage(leftFFVolts);
        //rightLeader.setVoltage(rightFFVolts);

        //this is a pid drive
        double leftRPM = Units.radiansPerSecondToRotationsPerMinute(leftVelocityRPS) * afterEncoderReduction;
        double rightRPM = Units.radiansPerSecondToRotationsPerMinute(rightVelocityRPS) * afterEncoderReduction;
        SmartDashboard.putNumber("LEFT RPM", leftRPM);
        SmartDashboard.putNumber("RIGHT RPM", rightRPM);

        //   Shuffleboard.getTab("Drive Details").add("LEFT RPM", leftRPM);
     //   Shuffleboard.getTab("Drive Details").add("RIGHT RPM", rightRPM);

        leftLeader.getPIDController().setReference(leftRPM, ControlType.kVelocity, 0, leftFFVolts,ArbFFUnits.kVoltage);
        rightLeader.getPIDController().setReference(rightRPM, ControlType.kVelocity, 0, rightFFVolts,ArbFFUnits.kVoltage);

    }

    /**
     * Handles deadband of the right stick
     * @param RY
     * @return
     */
    public double trueRightX(double RY) 
    {
        double stick = RY;
        stick *= Math.abs(stick);
        if (Math.abs(stick) < 0.1) {
            stick = 0;
        }
        return stick;
    }
    
    /**
     * handles deadband on the left stick
     * @param LY
     * @return
     */
    public double trueLeftX(double LY) 
    {
        double stick = LY;
        stick *= Math.abs(stick);
        if(Math.abs(stick) < 0.1) {
            stick = 0;
        }
        return stick;

    }

    /**
     * A basic stop command.
     */
    public void stop() {
        leftLeader.set(0);
        rightLeader.set(0);
    }

    /**
     * works to update the Odemetry details, limelight details etc...
     */
    @Override
    public void periodic()
    {
        odometry.update(new Rotation2d(Math.toRadians(gyro.getAngle())* -1),
        getLeftPositionMeters() - baseDistanceLeftRad,
        getRightPositionMeters() - baseDistanceRightRad);

        Pose2d robotPose = odometry.getPoseMeters();
        //log this 
       // new double  {robotPose.getX(), robotPose.getY(),
      //  robotPose.getRotation().getRadians()};
      // Shuffleboard.getTab("Drive").add()
        SmartDashboard.putNumber("LAST LEFT VELOCITY MPS", lastLeftVelocityMPS);

        SmartDashboard.putNumber("LAST RIGHT VELOCITY MPS", lastRightVelocityMPS);
        SmartDashboard.putNumber("LEFT ENCODER VELOCITY", leftEncoder.getVelocity());
        SmartDashboard.putNumber("RIGHT ENCODER VELOCITY", rightEncoder.getVelocity());
        SmartDashboard.putNumber("LEFT ENCODER POS", leftEncoder.getPosition());
        SmartDashboard.putNumber("RIGHT ENCODER POS", rightEncoder.getPosition());
        

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
       // SmartDashboard.putNumber("LimelightX", x);
       // SmartDashboard.putNumber("LimelightY", y);
       // SmartDashboard.putNumber("LimelightArea", area);
    }

    public double getLeftPositionMeters()
    {
        double lPosition = (leftInternalEncoder.getPosition() * (2.0 * Math.PI)
        /afterEncoderReduction) * Constants.WHEEL_RADIUS_METERS;
        return lPosition;
    }

    public double getRightPositionMeters()
    {
        double rPosition = (rightInternalEncoder.getPosition() * (2.0 * Math.PI)
        /afterEncoderReduction)*Constants.WHEEL_RADIUS_METERS;
        return rPosition;
    }

    
    public RelativeEncoder getRightEncoder()
    {
        return leftEncoder;
    }

    public RelativeEncoder getLeftEncoder(){
        return rightEncoder;
    }

    public SparkMaxPIDController getLeftPIDController() {
        return leftController;
    }

    public SparkMaxPIDController getRightPidController(){
        return rightController;
    }


    /**
   * Inverts NavX yaw as Odometry takes CCW as positive
   *
   * @return -180..180
   */
  public double getHeading() {
    double heading = -gyro.getYaw();
    if (heading > 180 || heading < 180) {
      heading = Math.IEEEremainder(heading, 360);
    }
    return heading;
  }

   
  
}

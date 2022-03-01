package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class Climber extends SubsystemBase{
    private VictorSPX climberBack;
    private TalonSRX climberLeft;
    private TalonSRX climberRight;
    	/** Tracking variables */
	boolean firstCall = false;
	boolean state = false;

    RelativeEncoder climbEncoder;



   public Climber(){
       climberBack = new VictorSPX(Constants.kClimberBack); //might swap this to a neo 550 so we get an encoder....
       climberLeft = new TalonSRX(Constants.kClimberLeft);
       climberRight = new TalonSRX(Constants.kClimberRight);
       climberLeft.configFactoryDefault();
       climberRight.configFactoryDefault();
       climberLeft.setNeutralMode(NeutralMode.Coast);
       climberRight.setNeutralMode(NeutralMode.Coast);
        /* Configure the left Talon's selected sensor as local QuadEncoder */
        climberLeft.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
        Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
        Constants.kTalonTimeoutMs);					// Configuration Timeout

        /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
        climberRight.configRemoteFeedbackFilter(climberLeft.getDeviceID(),					// Device ID of Source
        RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
        Constants.REMOTE_0,							// Source number [0, 1]
        Constants.kTalonTimeoutMs);						// Configuration Timeout
        /* Setup Sum signal to be used for Distance */
        climberRight.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTalonTimeoutMs);				// Feedback Device of Remote Talon
        climberRight.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonTimeoutMs);	// Quadrature Encoder of current Talon

        /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
        climberRight.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
                                                    Constants.PID_PRIMARY,
                                                    Constants.kTalonTimeoutMs);
        climberLeft.setInverted(false);
		climberLeft.setSensorPhase(true);
		climberRight.setInverted(true);
		climberRight.setSensorPhase(true);

        /* Set status frame periods to ensure we don't have stale data */
		climberRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTalonTimeoutMs);
		climberRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTalonTimeoutMs);
		climberRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTalonTimeoutMs);

        climberLeft.configPeakOutputForward(+1.0, Constants.kTalonTimeoutMs);
        climberLeft.configPeakOutputReverse(-1.0, Constants.kTalonTimeoutMs);
		climberRight.configPeakOutputForward(+1.0, Constants.kTalonTimeoutMs);
		climberRight.configPeakOutputReverse(-1.0, Constants.kTalonTimeoutMs);

        climberRight.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTalonTimeoutMs);
		climberRight.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTalonTimeoutMs);
		climberRight.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTalonTimeoutMs);
		climberRight.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTalonTimeoutMs);
		climberRight.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTalonTimeoutMs);
		climberRight.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput, Constants.kTalonTimeoutMs);
		climberRight.configAllowableClosedloopError(Constants.kSlot_Distanc, 0, Constants.kTalonTimeoutMs);
        /**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        int closedLoopTimeMs = 1;
        climberRight.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTalonTimeoutMs);

        climberRight.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTalonTimeoutMs);
		firstCall = true;
		state = false;
		zeroSensors();

        
   }


   /**set the main climber motors to coast */
   public void setCoast()
   {
    climberLeft.setNeutralMode(NeutralMode.Coast);
    climberRight.setNeutralMode(NeutralMode.Coast);
   }
   /**set the climber main motors to brake mode */
   public void setBrake()
   {
    climberLeft.setNeutralMode(NeutralMode.Brake);
    climberRight.setNeutralMode(NeutralMode.Brake);
   }

   /**used for the smaller motor on the back to raise the climber */
   public void raiseClimber(double climberSpeed)
   {
       climberBack.set(ControlMode.PercentOutput, climberSpeed);
   }

/** arms down climb*/
   public void lowClimb(double climberSpeed)
   {
       climberBack.set(ControlMode.PercentOutput,-1 * climberSpeed);
   }

   /**
    * Does not use a target distance - as we don't know it yet! But just a basic down 
    */
   public void simpleFrontLower(double climberSpeed){
    climberLeft.set(ControlMode.PercentOutput, climberSpeed);
    climberRight.set(ControlMode.PercentOutput, climberSpeed);

   }
   /**
    * Basic standard to losent the strings if in brake mode
    * @param climberSpeed
    */
   public void simpleFrontRaise(double climberSpeed){
    climberLeft.set(ControlMode.PercentOutput, -1 * climberSpeed);
    climberRight.set(ControlMode.PercentOutput, -1 * climberSpeed);

   }

   public void lowerClimber(double climberSpeed)
   {
   
        if(!state){
            if (firstCall)
            
            climberLeft.set(ControlMode.PercentOutput, Constants.kClimberSpeedFast, DemandType.ArbitraryFeedForward, +Constants.kClimberSpeed);
            climberLeft.set(ControlMode.PercentOutput, Constants.kClimberSpeedFast, DemandType.ArbitraryFeedForward, -Constants.kClimberSpeed);
        }
        else{
            if (firstCall) {
                System.out.println("This is Position Closed Loop with an Arbitrary Feed Forward.");
                System.out.println("Servo [-6, 6] rotations while having the ability to add a FeedForward with joyX.");
                zeroSensors();
                
                /* Determine which slot affects which PID */
                climberRight.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
            }
            
            /* Calculate targets from gamepad inputs */
            double target_sensorUnits = Constants.kClimberSpeedFast * Constants.kTalonSensorUnitsPerRotation * Constants.kClimberRotationsToTravel;
            double feedFwdTerm = Constants.kClimberSpeed * 0.10;	// Pecent to add to Closed Loop Output
            
            /* Configured for Position Closed Loop on Quad Encoders' Sum and Arbitrary FeedForward on joyX */
            climberRight.set(ControlMode.Position, target_sensorUnits, DemandType.ArbitraryFeedForward, feedFwdTerm);
            climberLeft.follow(climberRight);
        }
        
        firstCall = false;
   }

   public void stopClimber(){
       climberLeft.set(ControlMode.PercentOutput, 0.0);
       climberRight.set(ControlMode.PercentOutput, 0.0);
       climberBack.set(ControlMode.PercentOutput, 0.0);

   }
           
   public void zeroSensors() 
   {
    climberLeft.getSensorCollection().setQuadraturePosition(0, Constants.kTalonTimeoutMs);
    climberRight.getSensorCollection().setQuadraturePosition(0, Constants.kTalonTimeoutMs);
    }

   
}

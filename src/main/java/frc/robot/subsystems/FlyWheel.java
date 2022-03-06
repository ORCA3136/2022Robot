package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.models.PIDfly;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
public class FlyWheel extends PIDSubsystem {
    RelativeEncoder flyWheelEncoder;
    public CANSparkMax flyWheel1;
    private  CANSparkMax flyWheel2;
    private CANSparkMax flyWheel3;
    private final SimpleMotorFeedforward flyWheelFeedforward =
      new SimpleMotorFeedforward(
         Constants.kSVolts, Constants.kVVoltSecondsPerRotation);
    public FlyWheel(){
       super(new PIDController(PIDfly.kP, PIDfly.kI, PIDfly.kD));
       getController().setTolerance(Constants.kShooterToleranceRPS);
       ((Encoder) flyWheelEncoder).setDistancePerPulse(Constants.kEncoderDistancePerPulse);
       setSetpoint(Constants.kShooterTargetRPS);
       //declares them as CANSparkMaxes
       flyWheel1 = new CANSparkMax(Constants.FlyWheel1, MotorType.kBrushless);
       flyWheel2 = new CANSparkMax(Constants.FlyWheel2, MotorType.kBrushless);
       flyWheel3 = new CANSparkMax(Constants.FlyWheel3, MotorType.kBrushless);
       //encodes the CANs
       flyWheelEncoder = flyWheel1.getEncoder();
       flyWheelEncoder = flyWheel3.getEncoder();
       flyWheelEncoder = flyWheel2.getEncoder();
       //restores all factory defaults
       if(Constants.RESET_SPARKMAX){
         flyWheel1.restoreFactoryDefaults();
         flyWheel2.restoreFactoryDefaults();
         if(Constants.isFlyWheel3){
            flyWheel3.restoreFactoryDefaults();
         }
       }

       flyWheel2.setInverted(true);
       
       //max voltage
       flyWheel1.enableVoltageCompensation(12.0);
       flyWheel2.enableVoltageCompensation(12.0);
       flyWheel3.enableVoltageCompensation(12.0);
       //limits the max speed
       flyWheel1.setSmartCurrentLimit(82);
       flyWheel2.setSmartCurrentLimit(82);
       flyWheel3.setSmartCurrentLimit(82);
       //set the can timeout
       flyWheel1.setCANTimeout(0);
       flyWheel2.setCANTimeout(0);
       if(Constants.isFlyWheel3)
       {
         flyWheel3.setCANTimeout(0);
       }
       //flashes the sparkmaxes
       if (Constants.RESET_SPARKMAX)
       {
         flyWheel1.burnFlash();
         flyWheel2.burnFlash();
         if(Constants.isFlyWheel3){
            flyWheel3.burnFlash();
         }
       }
    }
   public void shoot(double flyWheelSpeed)
    {
       flyWheel1.set(flyWheelSpeed);
       flyWheel2.set(flyWheelSpeed);
       flyWheel3.set(flyWheelSpeed);
    }
    public void notShoot(double FlyWheelSpeed)
    {
      flyWheel1.set( -1 * FlyWheelSpeed);
      flyWheel2.set( -1 * FlyWheelSpeed);
      flyWheel3.set( -1 * FlyWheelSpeed);
    }
    public void stop()
    {
      flyWheel1.set(0);
      flyWheel2.set(0);
      flyWheel3.set(0);
    }
    @Override
    public void useOutput(double output, double setpoint) {
      flyWheel1.setVoltage(output + flyWheelFeedforward.calculate(setpoint));
      flyWheel2.setVoltage(output + flyWheelFeedforward.calculate(setpoint));
      flyWheel3.setVoltage(output + flyWheelFeedforward.calculate(setpoint));
    }
    @Override
    public double getMeasurement() {
      return ((Encoder) flyWheelEncoder).getRate();
     }
    public boolean atSetpoint() {
      return m_controller.atSetpoint();
    }
    public FlyWheel(PIDController controller) {
      super(controller);
      //TODO Auto-generated constructor stub
   }
}
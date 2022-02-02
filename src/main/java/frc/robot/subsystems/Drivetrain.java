package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax[] motors;
    private MotorControllerGroup right_motors;
    private MotorControllerGroup left_motors;
    private DifferentialDrive diffDrive;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private SparkMaxPIDController leftController;
    private SparkMaxPIDController rightController;

    public Drivetrain() {
        motors = new CANSparkMax[] { 
            new CANSparkMax(Constants.lm1_motor_id, MotorType.kBrushless),
            new CANSparkMax(Constants.lm2_motor_id, MotorType.kBrushless),
            new CANSparkMax(Constants.lm3_motor_id, MotorType.kBrushless),
            new CANSparkMax(Constants.rm1_motor_id, MotorType.kBrushless),
            new CANSparkMax(Constants.rm2_motor_id, MotorType.kBrushless),
            new CANSparkMax(Constants.rm3_motor_id, MotorType.kBrushless),
        };
        right_motors = new MotorControllerGroup(motors[3], motors[4], motors[5]);
        left_motors = new MotorControllerGroup(motors[0], motors[1], motors[2]);

            motors[1] . follow(motors[0]);
            motors[2] . follow(motors[0]);
            motors[4] . follow(motors[3]);
            motors[5] . follow(motors[3]);
            
            leftEncoder = motors[0].getEncoder();
            leftController = (SparkMaxPIDController) motors[0].getPIDController();
            ((SparkMaxPIDController) leftController) . setFeedbackDevice (leftEncoder);

            rightEncoder = motors[3].getEncoder();
            rightController = (SparkMaxPIDController) motors[3].getPIDController();
            ((SparkMaxPIDController) rightController) . setFeedbackDevice (rightEncoder);

            diffDrive = new DifferentialDrive(left_motors, right_motors);
    };

    public void drive( XboxController controller) 
  {
      diffDrive.tankDrive(trueRightX((controller.getLeftY() * Constants.kLeftDriveScaling)),
       trueRightX((controller.getRightY() * Constants.kLeftDriveScaling)), true);
      left_motors.set(trueLeftX((controller.getLeftY() * Constants.kLeftDriveScaling)));
     
  }

    public double trueRightX(double RY) {
        double stick = RY;
        stick *= Math.abs(stick);
        if (Math.abs(stick) < 0.1) {
            stick = 0;
        }
        return stick;
    }

    public double trueLeftX(double LY) {
        double stick = LY;
        stick *= Math.abs(stick);
        if(Math.abs(stick) < 0.1) {
            stick = 0;
        }
        return stick;

    }

    public void stop() {
        for (CANSparkMax t : motors) {
            t.set(0);
        }
    }

    @Override
    public void periodic(){

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
}

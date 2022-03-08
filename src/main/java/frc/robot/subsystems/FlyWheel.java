package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class FlyWheel extends SubsystemBase {
   RelativeEncoder flyWheelEncoder;
   RelativeEncoder flyWheelEncoder2;
   RelativeEncoder flyWheelEncoder3;

   private CANSparkMax flyWheel1;
   private CANSparkMax flyWheel2;
   private CANSparkMax flyWheel3;

   private SparkMaxPIDController flyWheel1PidController;
   private SparkMaxPIDController flyWheel2PidController;
   private SparkMaxPIDController flyWheel3PidController;
   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

   public FlyWheel() {
      // declares them as CANSparkMaxes
      flyWheel1 = new CANSparkMax(Constants.FLYWHEEL1, MotorType.kBrushless);
      flyWheel2 = new CANSparkMax(Constants.FLYWHEEL2, MotorType.kBrushless);
      flyWheel3 = new CANSparkMax(Constants.FLYWHEEL3, MotorType.kBrushless);
      // getting the encoders to mange RPMS
      flyWheelEncoder = flyWheel1.getEncoder();
      flyWheelEncoder2 = flyWheel2.getEncoder();
      flyWheelEncoder3 = flyWheel3.getEncoder();

      // restores all factory defaults
      if (Constants.RESET_SPARKMAX) {
         flyWheel1.restoreFactoryDefaults();
         flyWheel2.restoreFactoryDefaults();
         if (Constants.isFlyWheel3) {
            flyWheel3.restoreFactoryDefaults();
         }
      }

      // inverting flywhee 2 and 3
      flyWheel2.setInverted(true);
      flyWheel3.setInverted(true);

      // max voltage
      flyWheel1.enableVoltageCompensation(12.0);
      flyWheel2.enableVoltageCompensation(12.0);
      flyWheel3.enableVoltageCompensation(12.0);

      // limits the max speed
      flyWheel1.setSmartCurrentLimit(80);
      flyWheel2.setSmartCurrentLimit(80);
      flyWheel3.setSmartCurrentLimit(80);

      // set the can timeout
      flyWheel1.setCANTimeout(0);
      flyWheel2.setCANTimeout(0);
      if (Constants.isFlyWheel3) {
         flyWheel3.setCANTimeout(0);
      }
      // flashes the sparkmaxes
      if (Constants.RESET_SPARKMAX) {
         flyWheel1.burnFlash();
         flyWheel2.burnFlash();
         if (Constants.isFlyWheel3) {
            flyWheel3.burnFlash();
         }
      }

      // fetch the PID controllers
      flyWheel1PidController = flyWheel1.getPIDController();
      flyWheel2PidController = flyWheel2.getPIDController();
      flyWheel3PidController = flyWheel3.getPIDController();

      // some default PID values
      kP = 6e-5;
      kI = 0;
      kD = 0;
      kIz = 0;
      kFF = 0.000015;
      kMaxOutput = 1;
      kMinOutput = -1;
      maxRPM = 5700;

      // FlyWheel1 setup PId
      flyWheel1PidController.setP(kP);
      flyWheel1PidController.setI(kI);
      flyWheel1PidController.setD(kD);
      flyWheel1PidController.setIZone(kIz);
      flyWheel1PidController.setFF(kFF);
      flyWheel1PidController.setOutputRange(kMinOutput, kMaxOutput);

      // FlyWheel2 setup PID
      flyWheel2PidController.setP(kP);
      flyWheel2PidController.setI(kI);
      flyWheel2PidController.setD(kD);
      flyWheel2PidController.setIZone(kIz);
      flyWheel2PidController.setFF(kFF);
      flyWheel2PidController.setOutputRange(kMinOutput, kMaxOutput);

      // FlyWheel3 setup PID
      flyWheel3PidController.setP(kP);
      flyWheel3PidController.setI(kI);
      flyWheel3PidController.setD(kD);
      flyWheel3PidController.setIZone(kIz);
      flyWheel3PidController.setFF(kFF);
      flyWheel3PidController.setOutputRange(kMinOutput, kMaxOutput);

      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);

   }

   @Override
   public void periodic() {
      SmartDashboard.putNumber("FlyWheel1 Velocity", flyWheelEncoder.getVelocity());
      SmartDashboard.putNumber("FlyWheel3 Velocity", flyWheelEncoder3.getVelocity());
      SmartDashboard.putNumber("FlyWheel2 Velocity", flyWheelEncoder2.getVelocity());

      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);

      // if PID coefficients on SmartDashboard have changed, write new values to
      // controller TODO - possibly move these to methods, but once we tune we won't be changing
      //PID CONTROLER 1
      if ((p != kP)) {
         flyWheel1PidController.setP(p);
         kP = p;
      }
      if ((i != kI)) {
         flyWheel1PidController.setI(i);
         kI = i;
      }
      if ((d != kD)) {
         flyWheel1PidController.setD(d);
         kD = d;
      }
      if ((iz != kIz)) {
         flyWheel1PidController.setIZone(iz);
         kIz = iz;
      }
      if ((ff != kFF)) {
         flyWheel1PidController.setFF(ff);
         kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
         flyWheel1PidController.setOutputRange(min, max);
         kMinOutput = min;
         kMaxOutput = max;
      }
      //FLY Wheel 2 turning - frankly 1& 2 should be the same as they are both attached to flywheel
      if ((p != kP)) {
         flyWheel2PidController.setP(p);
         kP = p;
      }
      if ((i != kI)) {
         flyWheel2PidController.setI(i);
         kI = i;
      }
      if ((d != kD)) {
         flyWheel2PidController.setD(d);
         kD = d;
      }
      if ((iz != kIz)) {
         flyWheel2PidController.setIZone(iz);
         kIz = iz;
      }
      if ((ff != kFF)) {
         flyWheel2PidController.setFF(ff);
         kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
         flyWheel2PidController.setOutputRange(min, max);
         kMinOutput = min;
         kMaxOutput = max;
      }

      //realistically we will need a different set of values for flywheel 3 as it is a different beast.
      if ((p != kP)) {
         flyWheel3PidController.setP(p);
         kP = p;
      }
      if ((i != kI)) {
         flyWheel3PidController.setI(i);
         kI = i;
      }
      if ((d != kD)) {
         flyWheel3PidController.setD(d);
         kD = d;
      }
      if ((iz != kIz)) {
         flyWheel3PidController.setIZone(iz);
         kIz = iz;
      }
      if ((ff != kFF)) {
         flyWheel3PidController.setFF(ff);
         kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
         flyWheel3PidController.setOutputRange(min, max);
         kMinOutput = min;
         kMaxOutput = max;
      }

   }

   public void shoot(double flyWheelSpeed) {
      flyWheel1.set(flyWheelSpeed);
      // flyWheel2.set(flyWheelSpeed);
      // flyWheel3.set(flyWheelSpeed);
   }

   public void notShoot(double FlyWheelSpeed) {
      flyWheel1.set(-1 * FlyWheelSpeed);
      flyWheel2.set(-1 * FlyWheelSpeed);
      flyWheel3.set(-1 * FlyWheelSpeed);
   }

   public void stop() {
      flyWheel1.set(0);
      flyWheel2.set(0);
      flyWheel3.set(0);
   }

   public void PIDshoot(double setPoint) { //TODO add another variable for 3
      flyWheel1PidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      flyWheel2PidController.setReference(setPoint,CANSparkMax.ControlType.kVelocity);
      flyWheel3PidController.setReference(setPoint,CANSparkMax.ControlType.kVelocity); //TODO handle 3 separately

      SmartDashboard.putNumber("SetPoint", setPoint);
   }

}
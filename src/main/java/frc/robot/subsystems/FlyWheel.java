package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
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

   public double kP, kI, kD, kIz, kFF,kFF3, kP3, kI3,kMaxOutput, kMinOutput, maxRPM;

   public double f1SP = Constants.kShooterHighTargetRPS;
   public double f3SP = Constants.kShooterHighTargetF3RPS;

   public double D1SP = Constants.kShooterDistanceRPS;
   public double D3SP = Constants.kShooterDistancetF3RPS;


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
      //flyWheel2.follow(flyWheel1, true);
      flyWheel3.setInverted(true);

      //set to brake mode
      flyWheel1.setIdleMode(CANSparkMax.IdleMode.kBrake);
      flyWheel2.setIdleMode(CANSparkMax.IdleMode.kBrake);
      flyWheel3.setIdleMode(CANSparkMax.IdleMode.kBrake);
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
      kP = 0.0000001;
      kI = 0.000000012;
      kD = 0;
      kIz = 0;
      kFF = 0.000091;
      kMaxOutput = 1;
      kMinOutput = -1;
      maxRPM = 5700;
      kFF3 = 0.0001;
      kP3 = 0.0000001;
      kI3 = 0;


      flyWheel1PidController.setP(kP);
      flyWheel1PidController.setI(kI);
      flyWheel1PidController.setD(kD);
      flyWheel1PidController.setIZone(kIz);
      flyWheel1PidController.setFF(kFF);
      flyWheel1PidController.setOutputRange(kMinOutput, kMaxOutput);

      flyWheel2PidController.setP(kP);
      flyWheel2PidController.setI(kI);
      flyWheel2PidController.setD(kD);
      flyWheel2PidController.setIZone(kIz);
      flyWheel2PidController.setFF(kFF);
      flyWheel2PidController.setOutputRange(kMinOutput, kMaxOutput);
      
      flyWheel3PidController.setP(kP3);
      flyWheel3PidController.setI(0.00);
      flyWheel3PidController.setD(kD);
      flyWheel3PidController.setIZone(kIz);
      flyWheel3PidController.setFF(kFF3);
      flyWheel3PidController.setOutputRange(kMinOutput, kMaxOutput);
   }

   @Override
   public void periodic() {
      SmartDashboard.putNumber("FlyWheel1 Velocity", flyWheelEncoder.getVelocity());
      SmartDashboard.putNumber("FlyWheel3 Velocity", flyWheelEncoder3.getVelocity());
      SmartDashboard.putNumber("FlyWheel2 Velocity", flyWheelEncoder2.getVelocity());
      SmartDashboard.putNumber("FW1 Setpoint", f1SP);
      SmartDashboard.putNumber("FW3 Setpoint", f3SP);
      double FW1 = SmartDashboard.getNumber("FW1 Setpoint", 0);
      double FW3 = SmartDashboard.getNumber("FW3 Setpoint", 0);

      if ((FW1 != f1SP)) {
         f1SP = FW1;

      }
      if ((FW3!= f3SP)) {
         f3SP = FW3;
      }

            
      //TODO add booleans to display on dashboard
      
      /*
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
 

      if ((p != kP)) {
         flyWheel1PidController.setP(p);
         flyWheel2PidController.setP(p);
         //flyWheel3PidController.setP(p);
         kP = p;
      }
      if ((i != kI)) {
         flyWheel1PidController.setI(i);
         flyWheel2PidController.setI(i);
         //flyWheel3PidController.setI(i);

         kI = i;
      }
      if ((d != kD)) {
         flyWheel1PidController.setD(d);
         flyWheel2PidController.setD(d);
         //flyWheel3PidController.setD(d);

         kD = d;
      }
      if ((iz != kIz)) {
         flyWheel1PidController.setIZone(iz);
         flyWheel2PidController.setIZone(iz);
         //flyWheel3PidController.setIZone(iz);

         kIz = iz;
      }
      if ((ff != kFF)) {
         flyWheel1PidController.setFF(ff);
         flyWheel2PidController.setFF(ff);
         //flyWheel3PidController.setFF(ff);

         kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
         flyWheel1PidController.setOutputRange(min, max);
         flyWheel2PidController.setOutputRange(min, max);
         //flyWheel3PidController.setOutputRange(min, max);
         kMinOutput = min;
         kMaxOutput = max;
      }

     */   

   }

   public void shoot(double flyWheelSpeed) {
     flyWheel1.set(flyWheelSpeed);
      flyWheel2.set(flyWheelSpeed);
       flyWheel3.set(flyWheelSpeed);
   }

   public void notShoot(double FlyWheelSpeed) {
      flyWheel1.set(-1* FlyWheelSpeed);
      flyWheel2.set(-1 * FlyWheelSpeed);
      flyWheel3.set(-1 * FlyWheelSpeed);
   }

   public void kirbySuck(double FlyWheelSpeed) {
      flyWheel1.set(-1 * Constants.kFlyWheelSuck);
      flyWheel2.set(-1 * Constants.kFlyWheelSuck);
      flyWheel3.set(-1 * Constants.kFlyWheelThree);
   }

   public void notShootAuto(double FlyWheelSpeed) {
      flyWheel1.set(-.5* FlyWheelSpeed);
      flyWheel2.set(-.5 * FlyWheelSpeed);
      flyWheel3.set(-.5 * FlyWheelSpeed);
   }

   public void stop() {
      flyWheel1.set(0);
      flyWheel2.set(0);
      flyWheel3.set(0);
   }

   public void PIDshoot(double mainSetPoint, double flyWheel3Setpoint) {
      
      flyWheel1PidController.setP(kP);
      flyWheel1PidController.setI(kI);
      flyWheel1PidController.setD(kD);
      flyWheel1PidController.setIZone(kIz);
      flyWheel1PidController.setFF(kFF);
      flyWheel1PidController.setOutputRange(kMinOutput, kMaxOutput);

      flyWheel2PidController.setP(kP);
      flyWheel2PidController.setI(kI);
      flyWheel2PidController.setD(kD);
      flyWheel2PidController.setIZone(kIz);
      flyWheel2PidController.setFF(kFF);
      flyWheel2PidController.setOutputRange(kMinOutput, kMaxOutput);
      
      flyWheel3PidController.setP(.0000001);
      flyWheel3PidController.setI(0.00);
      flyWheel3PidController.setD(kD);
      flyWheel3PidController.setIZone(kIz);
      flyWheel3PidController.setFF(kFF3);
      flyWheel3PidController.setOutputRange(kMinOutput, kMaxOutput);
      

      flyWheel1PidController.setReference(f1SP, CANSparkMax.ControlType.kVelocity, 0, f1SP * kFF, ArbFFUnits.kPercentOut);
      flyWheel2PidController.setReference(f1SP, CANSparkMax.ControlType.kVelocity, 0, f1SP * kFF, ArbFFUnits.kPercentOut);
      flyWheel3PidController.setReference(f3SP, CANSparkMax.ControlType.kVelocity, 0, f3SP * kFF3, ArbFFUnits.kPercentOut);

      SmartDashboard.putNumber("SetPoint", mainSetPoint);
   }

   public void PIDdistance(double mainSetPoint, double flyWheel3Setpoint) {
      
      flyWheel1PidController.setP(kP);
      flyWheel1PidController.setI(kI);
      flyWheel1PidController.setD(kD);
      flyWheel1PidController.setIZone(kIz);
      flyWheel1PidController.setFF(kFF);
      flyWheel1PidController.setOutputRange(kMinOutput, kMaxOutput);

      flyWheel2PidController.setP(kP);
      flyWheel2PidController.setI(kI);
      flyWheel2PidController.setD(kD);
      flyWheel2PidController.setIZone(kIz);
      flyWheel2PidController.setFF(kFF);
      flyWheel2PidController.setOutputRange(kMinOutput, kMaxOutput);
      
      flyWheel3PidController.setP(.0000001);
      flyWheel3PidController.setI(0.00);
      flyWheel3PidController.setD(kD);
      flyWheel3PidController.setIZone(kIz);
      flyWheel3PidController.setFF(kFF3);
      flyWheel3PidController.setOutputRange(kMinOutput, kMaxOutput);
      

      flyWheel1PidController.setReference(D1SP, CANSparkMax.ControlType.kVelocity, 0, D1SP * kFF, ArbFFUnits.kPercentOut);
      flyWheel2PidController.setReference(D1SP, CANSparkMax.ControlType.kVelocity, 0, D1SP * kFF, ArbFFUnits.kPercentOut);
      flyWheel3PidController.setReference(D3SP, CANSparkMax.ControlType.kVelocity, 0, D3SP * kFF3, ArbFFUnits.kPercentOut);

      SmartDashboard.putNumber("SetPoint", mainSetPoint);
   }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.models.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
        //CAN ID MAPPING
        public static final int LEADER_LEFT_CAN_ID = 1;
        public static final int FOLLOWER_LEFT_CAN_ID = 2;
        public static final int FOLLOWER_LEFT_2_CAN_ID = 3;
    
        public static final int LEADER_RIGHT_CAN_ID = 4;
        public static final int FOLLOWER_RIGHT_CAN_ID = 5;
        public static final int FOLLOWER_RIGHT_2_CAN_ID = 6;
        
        public static final int FLYWHEEL1 = 7;
        public static final int FLYWHEEL2 = 8;
        public static final int FLYWHEEL3 = 12;
    
        public static boolean isFlyWheel3 = true;

        public static final int CONVEYOR = 9;
    
        public static final int INTAKE = 11;
        public static final int OUTERROLLER = 10;

        public static final int CLIMBERBACK = 15;
        public static final int CLIMBERLEFT = 16;
        public static final int CLIMBERRIGHT = 17;

        public static final int INTAKELEFT = 18;
        public static final int INTAKERIGHT = 19;


        //DRIVE MOTOR SETTINGS
        public static boolean RESET_SPARKMAX = true;

        public static double MAX_VELOCITY_MPS = Units.inchesToMeters(90.0);//upping the spped a bit to 8.33 //MAX IS around 13ft/sec (156 inches)
        public static double FS_MAX_VELOCITY_MPS = Units.inchesToMeters(150.0);//upping the spped a bit to 8.33 //MAX IS around 13ft/sec (156 inches)

        public static double loopPeriodSecs = 0.02;
        public static double WHEEL_RADIUS_METERS =  Units.inchesToMeters(3.0);
 
        //JOYTICK MAPPINGS
        public static final int kY = 4;
        public static final int kX = 1;
        public static final int kA = 2;
        public static final int kB = 5;
        public static final int kLB = 7;
        public static final int kRB = 3;
        public static final int kRT = 6;
        public static final int kLT = 8;
        public static final int kSelect = 11;
        public static final int kStart = 10;
        public static final int kHome = 12;

        //SPEED SETTINGS FOR SUBSYSTEMS    
        public static final double kConveyerHigh = .5;
        public static final double kConveyorLow = .3;

        public static final double kFlyWheelFast = - .75;
        public static final double kFlyWheelMedium = - .5;
        public static final double kFlyWheelSlow = - .34;
        public static final double kFlyWheelAuto = - .79;
        public static final double kFlyWheelSuck = - .20;
        public static final double kFlyWheelThree = 1;
    
        
        public static final double kIntakeHigh = .8;
        public static final double kIntakeLow = .3;

        public static final double kFancyIntakeHigh = .5;
        public static final double kFancyIntakeLow = .3;

        public static final double kLeftDriveScaling = .8;
        public static final double kRightDriveScaling = -.8;

 

        public static final double kLeftAuto = -.5;
        public static final double kRightAuto = -.5;
        public static final double kAutoDistance = 40;
        public static final double kAutoDistanceReverse = -40;
        public static final double kAutoDistance2 = 20;

        public static final double idealShoot = 60;

        //overly complex climber stuff....
        public static final double kClimberSpeed = .5;
        public static final double kClimberSpeedFast = 1;
        public static final double kClimberHold = .1;
       
        public final static int kTalonTimeoutMs = 30;
        public final static int kTalonSensorUnitsPerRotation = 4096;
        public final static double kClimberRotationsToTravel = 6;
        public final static int PID_PRIMARY = 0;
        public final static int REMOTE_0 = 0;
        public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
        public final static Gains kGains_Turning = new Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );
        public final static Gains kGains_Velocit = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
        public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );
        public final static int SLOT_0 = 0;
	    public final static int SLOT_1 = 1;
	    public final static int SLOT_2 = 2;
	    public final static int SLOT_3 = 3;
        public final static int kSlot_Distanc = SLOT_0;
        public final static int kSlot_Turning = SLOT_1;
        public final static int kSlot_Velocit = SLOT_2;
        public final static int kSlot_MotProf = SLOT_3;
        
        //flywheel PID testing
        public static final double kShooterHighTargetRPS = -2900;
        public static final double kShooterHighTargetF3RPS = 1160;

        public static final double kShooterHighTargetF3AutoRPS = 1400;
        public static final double kShooterHighTargetAutoTest = -2800;

        public static final double kMAXSHOTF3 = 3000;
        public static final double kMAXSHOT = -5000;

        public static final double kLaunchpadFlyWheel = -4000;
        public static final double kLaunchpadFlyWheel3 = 5550;

        public static final double kShooterLowTargetRPS = 1700;

        public static final double kShooterDistanceRPS = -2700;
        public static final double kShooterDistancetF3RPS = 2800;
        
}

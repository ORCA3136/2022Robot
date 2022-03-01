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
        //drivetrainmotores
        public static int LEADER_LEFT_CAN_ID = 1;
        public static int FOLLOWER_LEFT_CAN_ID = 2;
        public static int FOLLOWER_LEFT_2_CAN_ID = 3;
    
        public static int LEADER_RIGHT_CAN_ID = 4;
        public static int FOLLOWER_RIGHT_CAN_ID = 5;
        public static int FOLLOWER_RIGHT_2_CAN_ID = 6;
        
        public static int FlyWheel1 = 7;
        public static int FlyWheel2 = 8;
        public static int FlyWheel3 = 12;
    
        public static boolean isFlyWheel3 = false;

        public static int Climber1 = 10;

        public static int kConveyor1 = 9;
    
        
        public static final int kInnerIntake = 11;

        public static final int kClimberBack = 15;
        public static final int kClimberLeft = 16;
        public static final int kClimberRight = 17;


        public static boolean RESET_SPARKMAX = true;
    
        public static double MAX_VELOCITY_MPS = Units.inchesToMeters(80.0); //15feet/second converted to meters
        public static double loopPeriodSecs = 0.02;
        public static double WHEEL_RADIUS_METERS =  Units.inchesToMeters(3.0);

        public static double maxSpeed = .65;
    
    
        
    
      

        public static final int kY = 5;
        public static final int kX = 4;
        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kLB = 7;
        public static final int kRB = 8;
        public static final int kRT = 10;
        public static final int kLT = 9;
        public static final int kSelect = 11;
        public static final int kStart = 12;
        
        //Pneumatic Adressing    
        public static final double kConveyerHigh = .5;
        public static final double kConveyorLow = .3;

        public static final double kFlyWheelFast = - .85;
        public static final double kFlyWheelMedium = - .5;
        public static final double kFlyWheelSlow = - .34;
        public static final double kFlyWheelAuto = - .75;
        
        
        public static final int kIntakeForward = 3;
        public static final int kIntakeReverse = 0;
        
        public static final double kIntakeHigh = .8;
        public static final double kIntakeLow = .3;

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
        



}

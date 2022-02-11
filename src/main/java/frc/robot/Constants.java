// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
    
        public static boolean RESET_SPARKMAX = true;
    
        public static double MAX_VELOCITY_MPS = Units.inchesToMeters(100.0); //15feet/second converted to meters
        public static double loopPeriodSecs = 0.02;
        public static double WHEEL_RADIUS_METERS =  Units.inchesToMeters(3.0);
    
    
    public static int RightFlyWheel = 8;
    public static int LeftFlyWheel = 7;
    
    public static int kConveyor1 = 0;
    
    public static final int kIntake1 = 10;
    public static final int kIntake2 = 11;
    public static final int kIntake3 = 12;
    
    public final int kY = 0;
    public final int kX = 1;
    public final int kA = 2;
    public final int kB = 3;
    public final int kLB = 4;
    public final int kRB = 5;
    public final int kRT = 6;
    public final int kLT = 7;
    public final int kSelect = 8;
    public final int kStart = 9;
    
    //Pneumatic Adressing
    public static int kHopperStopperForward = 0;
    public static int kHopperStopperReverse = 1;
    
    public static final double kConveyorSpeed = .8;
    
    public static final double kFlyWheelSpeed = .8;
    
    
    public static final int kIntakeForward = 3;
    public static final int kIntakeReverse = 0;
    
    public static final double kIntakeHigh = .8;
    public static final double kIntakeLow = .3;

    public static final double kLeftDriveScaling = -.8;
     public static final double kRightDriveScaling = -.8;


}

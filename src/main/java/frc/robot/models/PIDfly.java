package frc.robot.models;
    public class PIDfly {
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kF;
    public static int kIzone;
    public static double kSetPointRPM;
    public PIDfly(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kSetpointRPM){
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        kIzone = _kIzone;
        kSetPointRPM = _kSetpointRPM;
    }};







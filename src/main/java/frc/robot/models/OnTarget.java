package frc.robot.models;


public class OnTarget {
    private double leftpower;

    public double getLeftPower(){
        return leftpower;
    }

    public double setLeftPower(double left){
         return this.leftpower = left;
    }

    public double rightpower;

    public double getRightPower(){
        return rightpower;
    }

    public double setRightPower(double right){
        return this.rightpower = right;
    }

    private double bigangle;

    public double getBigAngle(){
        return bigangle;
    }

    public double setBigAngle(double angle){
        return this.bigangle = angle;
    }

}



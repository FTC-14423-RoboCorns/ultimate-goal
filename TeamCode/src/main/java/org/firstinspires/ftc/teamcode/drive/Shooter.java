package org.firstinspires.ftc.teamcode.drive;

import android.app.ApplicationErrorReport;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public DcMotorEx shooter;
    public Servo pusher;
    public Servo lift;
    //TODO: confirm mag

    //the location of the red goal on the plane
    public static double goalX = 74;
    public static double goalY = -36;

    public static double powerShotY = -54;

    private static int SHOOTER_VELOCITY = 1900;
    private static int MAX_VELOCITY = 2160;
    private static double PUSHER_START=.16;
    private static double PUSHER_STOP=.43;
    private double RAMP_DISTANCE = 7.5;
    private double CRANK_RADIUS=1.111;
    private double ROD_LENGTH=3.75;
    private double SERVO_RANGE_ANGLE=Math.toRadians(160);

    //the location of the middle of the robot.
    public static double robotY = 0;
    public static double robotX = 0;

    //Is the robot on the red side(true) or blue side(false)
    public static boolean red = true;

    //height of the goal
    public static double goalHeight = 33.5;
    public static double powerShotHeight = 30.5;

    public target redGoal;
    public target blueGoal;

    public target redPowerShot1;
    public target redPowerShot2;
    public target redPowerShot3;

    public target bluePowerShot1;
    public target bluePowerShot2;
    public target bluePowerShot3;

    public Shooter(HardwareMap hardwareMap) {
        init(hardwareMap);
        redGoal= new target(goalX, goalY, goalHeight);
        blueGoal= new target(goalX, goalY*-1, goalHeight);

        redPowerShot1 = new target(goalX, powerShotY, goalHeight);
        redPowerShot2 = new target(goalX, powerShotY-8, goalHeight);
        redPowerShot3 = new target(goalX, powerShotY-16, goalHeight);

        bluePowerShot1 = new target(goalX, (powerShotY*-1), goalHeight);
        bluePowerShot2 = new target(goalX, (powerShotY*-1)-8, goalHeight);
        bluePowerShot3 = new target(goalX, (powerShotY*-1)-16, goalHeight);
    }

    private void init(HardwareMap hardwareMap) {
        //TODO: confirm mag class
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");
        lift= hardwareMap.get(Servo.class, "lift");
        lift.scaleRange(.12,.6);//lift servo now goes from 0,1
        lift.setPosition(.9);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pusher.setPosition(PUSHER_START);

    }


    public class target
    {
        public double x;
        public double y;
        public double height;
        public target(double thisX, double thisY, double thisHeight)
        {
            x=thisX;
            y=thisY;
            height= thisHeight;

        }
    }
    private double distanceToGoal(double robotX, double robotY, target goal)
    {
        //Finding the distance from the robot to the red goal
        double distance = 0;

            double x = robotX - goal.x;
            double y = robotY - goal.y;
            double xy = x*x + y*y;
            distance = Math.sqrt(xy);

        return distance;
    }

    public double shooterAngle(double robotX, double robotY, target goal)
    {
        //Finding the angle of the shooter relative to the goal
        double opposite = goal.height - 3;
        double adjacent = distanceToGoal(robotX - 8.5, robotY, goal);
        return Math.atan2(opposite, adjacent);
    }

    private double supportHeight(double angle)
    {
        //Finding the distance of the bottom of the ramp

        return RAMP_DISTANCE * Math.tan(angle);
    }

    //returns angle in RADIANS
    private double crankAngle(double height){
        double Lsquared = ROD_LENGTH*ROD_LENGTH;
        double Rsquared = CRANK_RADIUS*CRANK_RADIUS;
        double Xsquared = height * height;
        double numerator = Lsquared - Rsquared - Xsquared;
        double denominator=2*CRANK_RADIUS * height *-1;
        return Math.acos(numerator/denominator);
    }

    //assumes angle in radians
    private double liftPosFromAngle(double angle){
        return angle/SERVO_RANGE_ANGLE;
    }

    public double angleToGoal(double robotX, double robotY, target goal)
    {
        //Finding the angle of the robot it needs to turn to shoot to the goal
        double targetAngle = 0;

            double opposite = robotY - goal.y;
            double adjacent = robotX - goal.x;
            targetAngle = Math.atan2(opposite, adjacent);

        return targetAngle;
    }
    //three versions of this method
    //no parameter: go to default velocity
    //int parameter: sets velocity to the parameter
    //double parameter between 0 and 1: sets velocity to percent of max
    //returns the velocity set, as caller may not know default or max.
    public double shooterOn()
    {
        ((DcMotorEx) shooter).setVelocity(SHOOTER_VELOCITY);
        return SHOOTER_VELOCITY;
    }
    public double shooterOn(int velocity)
    {
        ((DcMotorEx) shooter).setVelocity(velocity);
        return velocity;
    }
    public double shooterOn(double percentOfMax)
    {
        if(percentOfMax>1)
        {
            percentOfMax=1;
        }
        else if(percentOfMax<0)
        {
            percentOfMax = 0;
        }
        ((DcMotorEx) shooter).setVelocity(percentOfMax*MAX_VELOCITY);
        return percentOfMax*MAX_VELOCITY;
    }

    public boolean isShooterReady(double velocity)
    {
        return shooter.getVelocity() >= velocity;
    }
    public void shooterOff()
    {
        ((DcMotorEx) shooter).setVelocity(0);
    }

    public void raiseShooter(double robotX, double robotY, target goal)
    {
        //range 0.6 (low) to 0.12 (high)
        double angle = shooterAngle(robotX, robotY, goal);
        double shooterHeight = supportHeight(angle);
        double crankAngle = crankAngle(shooterHeight);
        double liftPos = liftPosFromAngle(crankAngle);
        lift.setPosition(liftPos);


    }
    public void raiseShooterManual(double degrees)
    {
        //range 0.6 (low) to 0.12 (high)
        //double angle = currentAngle + degrees;
        //double shooterHeight = supportHeight(angle);
    //TODO need to get current angle and change it

    }

    public void lowerShooterManual(double degrees)
    {
        //range 0.6 (low) to 0.12 (high)
        //double angle = currentAngle - degrees;
        //double shooterHeight = supportHeight(angle);


    }

    public void pushRing()
    {
        pusher.setPosition(PUSHER_STOP);
        pusher.setPosition(PUSHER_START);
    }

    public void pusherIn(){
        pusher.setPosition(PUSHER_STOP);

    }
    public void pusherOut(){
        pusher.setPosition(PUSHER_START);
    }


}

package org.firstinspires.ftc.teamcode.drive;

import android.app.ApplicationErrorReport;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.util.Log.println;
import static java.lang.Math.cos;

public class Shooter {
    public DcMotorEx shooter;
    public Servo pusher;
    public Servo lift;
    public Telemetry telemetry;
    //TODO: confirm mag
    private boolean debug=false;
    public double angle;
    public double shooterHeight;
    public double crankAngle;
    public double liftPos;
    public boolean shooting;

    //the location of the red goal on the plane
    public final static double goalX = 72;
    public final static double goalY = -36;

    public boolean isShooterOn = false;
    public boolean isDoneShooting;

    public final static double powerShotY = -19;
    public double SHOOTER_OFFSET=7;
    private final int SHOOTER_VELOCITY = 2000;//2000;
    private final int MAX_VELOCITY = 2160;
    private final double PUSHER_START=.18;// was.20 on old servo;//was .16; testing faster time
    private final double PUSHER_STOP=.34;//was .41 on old servo;
    private final double RAMP_DISTANCE = 6;
    private final double CRANK_RADIUS=1.111;
    private final double ROD_LENGTH=3.75;
    private final double SERVO_RANGE_ANGLE=Math.toRadians(150);
    private final double RING_EXIT_VELOCITY=50;//feet per second
    private final double LAUNCH_HEIGHT=4;
    private final double GRAVITY_FEET=32.17;

    //the location of the middle of the robot.
    public static double robotY = 0;
    public static double robotX = 0;

    //Is the robot on the red side(true) or blue side(false)
    public static boolean red = true;

    //height of the goal
    public final double goalHeight = 33.5;
    public final double powerShotHeight = 30.5;

    public target redGoal;
    public target blueGoal;

    public target redPowerShot1;
    public target redPowerShot2;
    public target redPowerShot3;

    public target bluePowerShot1;
    public target bluePowerShot2;
    public target bluePowerShot3;

    public enum LiftState{
        DYNAMIC,
        STATIC
    }
    public LiftState liftState;

    public target currentTarget;

    public Shooter(HardwareMap hardwareMap, Telemetry telem) {
        init(hardwareMap);
        redGoal= new target(goalX, goalY, goalHeight);
        blueGoal= new target(goalX, goalY*-1, goalHeight);

        isShooterOn = false;
        liftState=LiftState.DYNAMIC;
        redPowerShot1 = new target(goalX-2, powerShotY, goalHeight);
        redPowerShot2 = new target(goalX-2, powerShotY+7.5, goalHeight);
        redPowerShot3 = new target(goalX-2, powerShotY+15, goalHeight);

        bluePowerShot1 = new target(goalX-2, (powerShotY*-1), goalHeight);
        bluePowerShot2 = new target(goalX-2, (powerShotY*-1)-7.5, goalHeight);
        bluePowerShot3 = new target(goalX-2, (powerShotY*-1)-15, goalHeight);

        this.telemetry = telem;
    }

    private void init(HardwareMap hardwareMap) {
        //TODO: confirm mag class
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");
        lift= hardwareMap.get(Servo.class, "lift");
        lift.scaleRange(.12,.6);//lift servo now goes from 0,1
        lift.setPosition(.6);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pusher.setPosition(PUSHER_START);
        currentTarget=redGoal;
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
        double opposite = (goal.height - LAUNCH_HEIGHT)/12;//delta y
        double adjacent = (distanceToGoal(robotX , robotY, goal))/12;//delta x
        //System.out.println("FTC Opposite "+ opposite);
        //System.out.println("FTC Adjacent "+ adjacent);

       //straight triangle method
        //return Math.atan2(opposite, adjacent);
        //trajectory method - as delta x gets smaller, this angle approaches the atan method.
        double v_squared = RING_EXIT_VELOCITY*RING_EXIT_VELOCITY;
        //System.out.println("ShooterAngle_FTC v_squared "+ v_squared);
        double first_term = (v_squared)/(GRAVITY_FEET*adjacent);
        //System.out.println("ShooterAngle_FTC first_term "+ first_term);
        double numerator = v_squared*(v_squared-(2*GRAVITY_FEET*opposite));
        //System.out.println("ShooterAngle_FTC numerator "+ numerator);
        double denominator = (GRAVITY_FEET*GRAVITY_FEET)*(adjacent*adjacent);
        //System.out.println("ShooterAngle_FTC denominator "+ denominator);
        double second_term = Math.sqrt((numerator/denominator)-1);
        //System.out.println("ShooterAngle_FTC second_term "+ second_term);
        //System.out.println("ShooterAngle_FTC final "+ (first_term-second_term));
        return Math.atan(first_term-second_term);


    }

    private double supportHeight(double angle)
    {
        //Finding the distance of the bottom of the ramp
        double tempHeight = RAMP_DISTANCE * Math.tan(angle);

       // return Range.clip(tempHeight,ROD_LENGTH-CRANK_RADIUS,ROD_LENGTH+CRANK_RADIUS);
        return tempHeight;
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
        //double targetAngle = 0;

            //double opposite = robotY - goal.y;
            //double adjacent = robotX - goal.x;
            //targetAngle = Math.atan2(opposite, adjacent);
        double distance = distanceToGoal(robotX,robotY,goal);
        double a = Math.acos(SHOOTER_OFFSET/distance);
        Vector2d goalVec = new Vector2d(goal.x, goal.y);
        Vector2d botVec = new Vector2d(robotX, robotY);
        Vector2d difference = goalVec.minus(botVec);
        double b = difference.angle();
        double t = b+a;
        double tanx=robotX+(SHOOTER_OFFSET*Math.cos(a));
        double tany=robotY+(SHOOTER_OFFSET*Math.sin(a));
        Vector2d tanVec=new Vector2d(tanx,tany);
        Vector2d difference2=goalVec.minus(tanVec);
        double theta = difference2.angle();


        /* old angle code
        Vector2d goalVec = new Vector2d(goal.x, goal.y);
        Vector2d botVec = new Vector2d(robotX, robotY+SHOOTER_OFFSET);
        Vector2d difference = goalVec.minus(botVec);

        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angle();

         */
       if (debug) System.out.println("SHOOTER_TURN_angle to goal absolute " + Math.toDegrees(theta));
        return theta;
    }

    public double angleToGoal(Vector2d vec, target goal){
         return angleToGoal(vec.getX(),vec.getY(),goal);
    }

    //three versions of this method
    //no parameter: go to default velocity
    //int parameter: sets velocity to the parameter
    //double parameter between 0 and 1: sets velocity to percent of max
    //returns the velocity set, as caller may not know default or max.
    public double shooterOn()
    {
        ((DcMotorEx) shooter).setVelocity(SHOOTER_VELOCITY);
        isShooterOn = true;
        return SHOOTER_VELOCITY;
    }
    public double shooterOn(int velocity)
    {
        ((DcMotorEx) shooter).setVelocity(velocity);
        isShooterOn = true;
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
        isShooterOn = true;
        return percentOfMax*MAX_VELOCITY;
    }

    public boolean isShooterReady(double velocity)
    {
        return shooter.getVelocity() >= velocity;
    }

    public void shooterOff()
    {
        ((DcMotorEx) shooter).setVelocity(0);
        isShooterOn = false;
    }

    public void raiseShooter(double robotX, double robotY, target goal)
    {
        //range 0.6 (low) to 0.12 (high)
        /*angle = shooterAngle(robotX, robotY, goal);
        System.out.println("Raise_FTC angle "+ angle);
        shooterHeight = supportHeight(angle);
        System.out.println("Raise_FTC shooterHeight "+ shooterHeight);
        crankAngle = crankAngle(shooterHeight+1.25);
        System.out.println("Raise_FTC crank angle "+ crankAngle);
        liftPos = liftPosFromAngle(crankAngle);
        System.out.println("Raise_FTC liftPos "+ liftPos);
        //lift.setPosition(liftPos);*/

        switch (liftState) {

        case STATIC:
            if (Math.abs(goal.y) < 26) {
                if (Math.abs(robotX)<24) liftPos = .68;//0.7;
                else liftPos = .76;
            } else {
               // liftPos = .58;//0.54;
                if (Math.abs(robotX)<24) liftPos = .56;//.58
                else
                liftPos=.70;//.71;
            }
            break;
        case DYNAMIC:
        double distance = (distanceToGoal(robotX, robotY, goal));
        //System.out.println("SHOOTER_TURN_distance: "+distance);
        if (Math.abs(goal.y) < 24) {
           // liftPos = 0.61;
            liftPos=.68;//0.64;
        } else {

            if (distance < 78) {
                liftPos =.58;// .56;//.54;
            } else if (distance < 83) {
                liftPos = 0.57;//.57
            } else if (distance < 88) {
                liftPos = 0.57;
            } else if (distance < 93) {
                liftPos = 0.63;
            } else if (distance < 98) {
                liftPos = 0.64;
            } else if (distance < 103) {
                liftPos = 0.64;
            } else if (distance < 108) {
                liftPos = 0.65;
            } else if (distance < 113) {
                liftPos = 0.65;
            } else if (distance < 118) {
                liftPos = 0.64;
            } else if (distance < 123) {
                liftPos = 0.64;
            }
        }
        break;

    }
        lift.setPosition(liftPos);

    }
    public void raiseShooterManual(double degrees)
    {
        //range 0.6 (low) to 0.12 (high)
        //double angle = currentAngle + degrees;
        //double shooterHeight = supportHeight(angle);
    //TODO need to get current angle and change it
        lift.setPosition(lift.getPosition()+.01);

    }

    public void lowerShooterManual(double degrees)
    {
        //range 0.6 (low) to 0.12 (high)
        //double angle = currentAngle - degrees;
        //double shooterHeight = supportHeight(angle);
        //TODO need to get current angle and change it
        lift.setPosition(lift.getPosition()-.01);

    }
/*
    public void pushRing()
    {
        pusher.setPosition(PUSHER_STOP);
        pusher.setPosition(PUSHER_START);
    }
*/
    public void pusherIn(){
        pusher.setPosition(PUSHER_STOP);
        shooting = true;


    }
    public void pusherOut(){
        pusher.setPosition(PUSHER_START);
        shooting = false;
    }

    public void update(Pose2d position){
        raiseShooter(position.getX(),position.getY(),currentTarget);

    }

}

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public DcMotorEx shooter;
    public CRServo magazine;
    //TODO: Add lift and confirm mag

    //the location of the red goal on the plane
    public static double redGoalX = 74;
    public static double redGoalY = -36;

    //the location of the blue goal on the plane
    public static double blueGoalX = 0;
    public static double blueGoalY = 0;

    //the location of the middle of the robot.
    public static double robotY = 0;
    public static double robotX = 0;

    //Is the robot on the red side(true) or blue side(false)
    public static boolean red = true;

    //height of the goal
    public static double goalHeight = 33.5;

    public Shooter(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        //TODO: Add lift and confirm mag class
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        magazine = hardwareMap.get(CRServo.class, "Magazine");
    }

    private static double distanceToGoal(double robotX, double robotY, boolean red)
    {
        //Finding the distance from the robot to the red goal
        double distance = 0;

        if(red = true)
        {
            double x = robotX - redGoalX;
            double y = robotY - redGoalY;
            double xy = x*x + y*y;
            distance = Math.sqrt(xy);
        }

        return distance;
    }

    public static double shooterAngle(double robotX, double robotY, boolean red)
    {
        //Finding the angle of the shooter relative to the goal
        double opposite = goalHeight - 3;
        double adjacent = distanceToGoal(robotX - 8.5, robotY, red);
        double angle = Math.atan2(opposite, adjacent);
        return angle;
    }

    public static double supportHeight(double angle)
    {
        //Finding the distance of the bottom of the ramp
        double rampDistance = 7.5;
        double height = rampDistance * Math.tan(angle);
        return height;
    }

    public double angleToGoal(double robotX, double robotY, boolean red)
    {
        //Finding the angle of the robot it needs to turn to shoot to the goal
        double goalAngle = 0;

        if(red = true)
        {
            double opposite = redGoalY - robotY;
            double adjacent = redGoalX - robotX;
            goalAngle = Math.atan2(opposite, adjacent);

        }
        return goalAngle;
    }

}
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.Math.*;

import org.firstinspires.ftc.teamcode.drive.UltimateMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class SkeletonOpMode extends LinearOpMode {

    //the location of the red goal on the plane
    double redGoalX = 0;
    double redGoalY = 0;

    //the location of the blue goal on the plane
    double blueGoalX = 0;
    double blueGoalY = 0;

    //the location of the middle of the robot.
    double robotY = 0;
    double robotX = 0;

    //Is the robot on the red side(true) or blue side(false)
    boolean red = true;

    //height of the goal
    double goalHeight = 33.5;
    @Override
    public void runOpMode() throws InterruptedException
    {
        UltimateMecanumDrive drive = new UltimateMecanumDrive(hardwareMap);
        

        waitForStart();


        if (isStopRequested()) return;
    }

    public double distanceToGoal(double robotX, double robotY, boolean red)
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

    public double shooterAngle(double robotX, double robotY, boolean red)
    {
        //Finding the angle of the shooter relative to the goal
            double opposite = goalHeight - 3;
            double adjacent = distanceToGoal(robotX - 8.5, robotY, red);
            double angle = Math.atan2(opposite, adjacent);
            return angle;
    }

    public double supportHeight(double angle)
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
            double opposite = robotY - redGoalY;
            double adjacent = robotX - redGoalX;
            goalAngle = Math.atan2(opposite, adjacent);

        }
        return goalAngle;
    }

}

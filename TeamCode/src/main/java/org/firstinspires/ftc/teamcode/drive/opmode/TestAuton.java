package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Shooter;
import org.firstinspires.ftc.teamcode.drive.Robot;
/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TestAuton extends LinearOpMode {
    public static double DISTANCE = 60; // in

    public Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {


        int isRed = 1; // BLUE = -1; RED = 1
        int ringPosition;
        robot = new Robot(hardwareMap);

        Pose2d startInsidePose = new Pose2d(-63, -24 * isRed, 0);
        //Pose2d startOutsidePose = new Pose2d(X, Y * isRed, 0);

        robot.drive.setPoseEstimate(startInsidePose);

        Trajectory trajectory1 = robot.drive.trajectoryBuilder(startInsidePose)
                .splineTo(new Vector2d(0, isRed * -12), Shooter.shooterAngle(0, -12, robot.shooter.redPowerShot1))
                .build();
        Trajectory trajectory2 = robot.drive.trajectoryBuilder(trajectory1.end())
                .splineTo(new Vector2d(60, isRed * -55), 0)
                .build();
        Trajectory trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                //.splineTo(new Vector2d(-55, isRed * -55), 0)
                .back(115)
                .build();
        Trajectory trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end())
                //.splineTo(new Vector2d(60, isRed * -55), 0)
                .forward(115)
                .build();
        Trajectory trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end())
                //.splineTo(new Vector2d(12, isRed * -55), 0)
                .back(48)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        /*robot.drive.followTrajectory(trajectory1);
        sleep(1000);
        robot.drive.followTrajectory(trajectory2);
        sleep(1000);
        robot.drive.followTrajectory(trajectory3);
        sleep(1000);
        robot.drive.followTrajectory(trajectory4);
        sleep(1000);
        robot.drive.followTrajectory(trajectory5);
        */

        while (!isStopRequested()) {
            ringPosition = robot.pixy.getStackHeight();
            telemetry.addData("Stack Height", ringPosition);
            telemetry.addData("key", robot.pixy.sensorHeight);
            telemetry.addData("key", robot.pixy.oneRing);
            telemetry.update();
        }

    }

}

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Shooter;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TestAuton extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startBlueInsidePose = new Pose2d(-63, 24, 0); //For blue, mirror red's y values
        Pose2d startRedInsidePose = new Pose2d(-63, -24, 0);



        drive.setPoseEstimate(startRedInsidePose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startRedInsidePose)
                .splineTo(new Vector2d(0, -12), Shooter.shooterAngle(0, -12, true))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineTo(new Vector2d(60, -55), 0)
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                //.splineTo(new Vector2d(-55, -55), 0)
                .back(115)
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                //.splineTo(new Vector2d(60, -55), 0)
                .forward(115)
                .build();
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                //.splineTo(new Vector2d(12, -55), 0)
                .back(48)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory1);
        sleep(1000);
        drive.followTrajectory(trajectory2);
        sleep(1000);
        drive.followTrajectory(trajectory3);
        sleep(1000);
        drive.followTrajectory(trajectory4);
        sleep(1000);
        drive.followTrajectory(trajectory5);


    }
}

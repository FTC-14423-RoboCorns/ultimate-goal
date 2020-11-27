package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.UltimateMecanumDrive;

@Config
@Autonomous(group = "drive")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        UltimateMecanumDrive drive = new UltimateMecanumDrive(hardwareMap);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectoryBackward = new TrajectoryBuilder(trajectoryForward.end(),trajectoryForward.end().getHeading(),
                DriveConstants.BASE_CONSTRAINTS)
                .back(DISTANCE)
                .build();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryBackward);
        }
    }
}

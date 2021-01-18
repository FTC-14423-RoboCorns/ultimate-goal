package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Shooter;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.advanced.AsyncFollowingFSM;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TestAuton extends OpMode {
    public static double DISTANCE = 60; // in

    int isRed; // BLUE = -1; RED = 1
    int shootCount = 0;
    double targetVelocity = 1900;
    boolean shooterReady;
    public Robot robot;
    int ringPosition;
    Trajectory trajectory1,trajectory2,trajectory3,trajectory4,trajectory5;
    Pose2d startPose;
    enum StartPosEnum {
        INSIDE_RED,
        OUTSIDE_RED,
        INSIDE_BLUE,
        OUTSIDE_BLUE
    }

    StartPosEnum startPos;


    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        SHOOTER_ON,
        SHOOT,
        TURN,
        DRIVE_WOBBLE_1,
        DROP_WOBBLE_1
        //TURN_2,         // Finally, we're gonna turn again
        //IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState;
    @Override

    public void init(){
        //TODO: Create Pre-Match Selection Using Joystick
        startPos = StartPosEnum.INSIDE_RED;

        robot = new Robot(hardwareMap, telemetry);
        switch (startPos){
            case INSIDE_RED:

                isRed = 1;
                startPose = new Pose2d(-63, -24 * isRed, 0);
                break;

            case INSIDE_BLUE:

                isRed = -1;
                startPose = new Pose2d(-63, -24 * isRed, 0);
                break;

        }

        robot.shooter.pusherOut();

        //Pose2d startOutsidePose = new Pose2d(X, Y * isRed, 0);

        robot.drive.setPoseEstimate(startPose);

        trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, isRed * -12), robot.shooter.shooterAngle(0, -12, robot.shooter.redPowerShot1))
                .build();
        robot.shooter.currentTarget=robot.shooter.redPowerShot1;
        robot.shooter.update(robot.drive.getPoseEstimate());
        PoseStorage.currentPose = robot.drive.getPoseEstimate();
        currentState=State.TRAJECTORY_1;
    }

    public void init_loop(){
        ringPosition = robot.pixy.getStackHeight();
        telemetry.addData("Stack Height", ringPosition);
        telemetry.addData("key", robot.pixy.sensorHeight);
        telemetry.addData("key", robot.pixy.oneRing);
        telemetry.update();
    }

    public void start(){
        ringPosition = robot.pixy.getStackHeight();
        telemetry.addData("Stack Height", ringPosition);
        telemetry.addData("key", robot.pixy.sensorHeight);
        telemetry.addData("key", robot.pixy.oneRing);
        telemetry.update();

        robot.drive.followTrajectoryAsync(trajectory1);
    }

    public void loop() {

        ElapsedTime waitTimer1 = new ElapsedTime();
        switch (currentState) {

            case TRAJECTORY_1:

                if (!robot.drive.isBusy()) {
                    currentState = State.SHOOTER_ON;
                }
                break;

            case SHOOTER_ON:
                if (!robot.shooter.isShooterOn) {
                    targetVelocity = robot.shooter.shooterOn();
                }
                if (robot.shooter.isShooterReady(targetVelocity)) {
                    currentState = State.SHOOT;
                } //will need to add a timer later to move on in case we never get up to speed
                break;

            case SHOOT:
                if (!robot.drive.isBusy()) {//making sure our turn is done
                    if(shootCount<4)
                    {
                        robot.shooter.pusherIn();
                        shootCount += 1;
                    }
                    if (shootCount < 3) {
                        currentState = State.TURN;
                    }
                    else
                    {
                        if (!robot.shooter.isShooterReady(targetVelocity-200)) {
                            robot.shooter.shooterOff();
                            robot.shooter.pusherOut();
                            currentState = State.DRIVE_WOBBLE_1;
                        }
                    }
                }
                break;

            case TURN:
                if (!robot.shooter.isShooterReady(targetVelocity)) {
                    robot.shooter.pusherOut();
                    powerTurn();
                    currentState = State.SHOOTER_ON;
                }
            case DRIVE_WOBBLE_1:
                setDriveWobble1();
                robot.drive.followTrajectoryAsync(trajectory2);
                currentState = State.DROP_WOBBLE_1;
            case DROP_WOBBLE_1:
                //add code here to drop the wobble
        }


        /*
        sleep(1000);
        robot.drive.followTrajectoryAsync(trajectory2);
        sleep(1000);
        robot.drive.followTrajectoryAsync(trajectory3);
        sleep(1000);
        robot.drive.followTrajectoryAsync(trajectory4);
        sleep(1000);
        robot.drive.followTrajectoryAsync(trajectory5);
        */

            robot.drive.update();
            PoseStorage.currentPose = robot.drive.getPoseEstimate();
            robot.shooter.update(robot.drive.getPoseEstimate());
            telemetry.addData("Stack Height", ringPosition);
            telemetry.addData("key", robot.pixy.sensorHeight);
            telemetry.addData("key", robot.pixy.oneRing);
            telemetry.update();

    }

    public void turnTo(double targetAngle, boolean isInputRadians)
    {
        double currentHeading = robot.drive.getPoseEstimate().getHeading();

        if(!isInputRadians)
        {
           targetAngle = Math.toRadians(targetAngle);
        }

        robot.drive.turnAsync(targetAngle - currentHeading);
    }
    public void turnTo(double targetAngle)
    {
        turnTo(targetAngle, true);
    }

    public void powerTurn()
    {
        if(isRed == 1)
        {
            if(shootCount == 1)
            {
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot2));
            }
            else
            {
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot3));
            }
        }
        else
        {
            if(shootCount == 1)
            {
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.bluePowerShot2));
            }
            else
            {
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.bluePowerShot3));
            }
        }
    }

    public void setDriveWobble1()
    {

        switch (ringPosition)
        {
            case 0:
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .splineTo(new Vector2d(60, isRed * -55), 0)
                        .build();
                break;
            case 1:
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .splineTo(new Vector2d(60, isRed * -55), 0)
                        .build();
                break;
            case 2:
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory2 = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .splineTo(new Vector2d(60, isRed * -55), 0)
                        .build();
                break;
        }
        trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                //.splineTo(new Vector2d(-55, isRed * -55), 0)
                .back(115)
                .build();
        trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end())
                //.splineTo(new Vector2d(60, isRed * -55), 0)
                .forward(115)
                .build();
        trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end())
                //.splineTo(new Vector2d(12, isRed * -55), 0)
                .back(48)
                .build();
    }


}

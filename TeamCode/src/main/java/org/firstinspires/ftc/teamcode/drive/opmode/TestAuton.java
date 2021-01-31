package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TestAuton extends OpMode {
    public static double DISTANCE = 60; // in
    double PowerTarget;
    int isRed; // BLUE = -1; RED = 1
    int shootCount = 0;
    double targetVelocity = 1900;
    boolean shooterReady;
    public Robot robot;
    int ringPosition;
    int wobblePos = 0;
    Trajectory trajectory1,trajectory2,trajectory3,trajectory4,trajectory5,misswobble;
    Pose2d startPose;
    ElapsedTime waitTimer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime wobbleWait = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    enum StartPosEnum {
        INSIDE_RED,
        OUTSIDE_RED,
        INSIDE_BLUE,
        OUTSIDE_BLUE
    }
    enum WobbleState {
        WOBBLE_RAISE,
        WOBBLE_RAISEWAIT,
        WOBBLE_LOWER,
        WOBBLE_LOWERWAIT,
        WOBBLE_OFF
    }

    StartPosEnum startPos;
    WobbleState wobbleState;


    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        FIRST_TURN,
        SHOOTER_ON,
        SHOOT,
        TURN,
        DRIVE_WOBBLE_1,
        DROP_WOBBLE_1,
        WOBBLE_DOWN,
        GET_WOBBLE_2,
        GRAB_WOBBLE_2,
        GRAB_WAIT,
        DRIVE_WOBBLE_2,
        DROP_WOBBLE_2,
        CLAW_WAIT,
        PARK,
        OFF
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
                PowerTarget =363;

                break;

            case INSIDE_BLUE:

                isRed = -1;
                startPose = new Pose2d(-63, -24 * isRed, 0);
                PowerTarget =1;
                break;

        }
        PoseStorage.isRed=isRed;
        robot.shooter.pusherOut();

        //Pose2d startOutsidePose = new Pose2d(X, Y * isRed, 0);

        robot.drive.setPoseEstimate(startPose);

        trajectory1 = robot.drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)), -15)
                //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
                .build();
        robot.shooter.currentTarget=robot.shooter.redPowerShot1;
        robot.shooter.update(robot.drive.getPoseEstimate());
        PoseStorage.currentPose = robot.drive.getPoseEstimate();
        currentState=State.TRAJECTORY_1;

        robot.wobble.closeClaw();
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
        wobblePos = 450;
        wobbleState = WobbleState.WOBBLE_RAISE;
    }

    public void loop() {

        switch (currentState) {
            //TODO: CHANGE ORDER FOR OUTSIDE RED
            case TRAJECTORY_1:
                if (!robot.drive.isBusy()) {
                    turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1));
 //               System.out.println("SHOOTER_firstAngle " + Math.toDegrees(robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)));
                }
                break;

            case FIRST_TURN:
                if (!robot.drive.isBusy()){
                    currentState = State.SHOOTER_ON;
                }
                break;

            case SHOOTER_ON:
                System.out.println("SHOOTER_shooterOn");
                if (!robot.shooter.isShooterOn) {
                    targetVelocity = robot.shooter.shooterOn();
                }
                if (robot.shooter.isShooterReady(targetVelocity)) {
                    waitTimer1.reset();
                    currentState = State.SHOOT;
                } //will need to add a timer later to move on in case we never get up to speed
                break;

            case SHOOT:
                boolean done;
                System.out.println("SHOOTER_shootInState");
                System.out.println("SHOOTER_shootInState still turning " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                if (!robot.drive.isBusy()&& waitTimer1.time()>1000) {//making sure our turn is done  && waitTimer1.time()>1500
                   /* if (isRed==1) {
                        done=robot.drive.getPoseEstimate().getHeading() <= Math.toRadians(PowerTarget);
                    } else {
                        done=robot.drive.getPoseEstimate().getHeading()>= Math.toRadians(PowerTarget);
                    }
                    if (done) {*/
                    //System.out.println("SHOOTER_now shooting heading " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                    if(shootCount<4)
                    {
                        System.out.println("SHOOTER_shoot " + shootCount);
                        System.out.println("SHOOTER_Final heading" + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                        robot.shooter.pusherIn();
                        shootCount += 1;
                    }
                    if (shootCount < 3) {
                        System.out.println("SHOOTER_shootToTurn");
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
                System.out.println("SHOOTER_turnInState");
                if (!robot.shooter.isShooterReady(targetVelocity-200)) {
                    System.out.println("SHOOTER_ringShot");
                    robot.shooter.pusherOut();
                    powerTurn();
                    currentState = State.SHOOTER_ON;
                }
                break;

            case DRIVE_WOBBLE_1:
                wobblePos=630;
                setDriveWobble1();
                robot.drive.followTrajectoryAsync(trajectory2);
                currentState = State.DROP_WOBBLE_1;
            case DROP_WOBBLE_1:
                //add code here to drop the wobble
                wobblePos=850;
                if(!robot.drive.isBusy()) {
                    waitTimer1.reset();
                    wobblePos=850;
                    robot.wobble.openClaw();
                    currentState = State.GET_WOBBLE_2;
                }
                break;
            case GET_WOBBLE_2:
                if(waitTimer1.time()>1000)
                {
                    wobbleState = WobbleState.WOBBLE_LOWER;
                    wobblePos = 450;
                    robot.drive.followTrajectoryAsync(trajectory3);
                    currentState = State.WOBBLE_DOWN;
                    waitTimer1.reset();
                }
                break;
            case WOBBLE_DOWN:
                if(waitTimer1.time()>1000)
                {
                    wobbleState = WobbleState.WOBBLE_RAISE;
                    wobblePos = 850;
                    currentState = State.GRAB_WOBBLE_2;
                }
                break;

            case GRAB_WOBBLE_2:
                if (!robot.drive.isBusy()) {
                    robot.wobble.closeClaw();
                    waitTimer1.reset();
                    currentState = State.GRAB_WAIT;
                }
                break;
            case GRAB_WAIT:
                if (waitTimer1.time()>250) {
                    robot.drive.followTrajectoryAsync(trajectory4);
                    wobbleState = WobbleState.WOBBLE_LOWER;
                    wobblePos = 450;
                    currentState = State.DRIVE_WOBBLE_2;
                    waitTimer1.reset();
                }
                break;

            case DRIVE_WOBBLE_2:
                if (waitTimer1.time()>1500) {
                    wobbleState = WobbleState.WOBBLE_RAISE;
                    wobblePos = 630;
                    currentState = State.DROP_WOBBLE_2;
                }
                break;
            case DROP_WOBBLE_2:
                wobblePos=850;
                if(!robot.drive.isBusy()) {
                    wobblePos = 850;
                    robot.wobble.openClaw();
                    currentState = State.CLAW_WAIT;
                    waitTimer1.reset();
                }
                break;
            case CLAW_WAIT:
                if (waitTimer1.milliseconds() > 500){
                    currentState = State.PARK;
                }

                break;

            case PARK:
                wobbleState = WobbleState.WOBBLE_LOWER;
                wobblePos = 0;
                robot.drive.followTrajectoryAsync(trajectory5);
                currentState = State.OFF;
                break;
            case OFF:
                wobbleState = WobbleState.WOBBLE_OFF;
                break;
        }

        handleWobble();

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

    public void handleWobble() {
        switch (wobbleState)
        {
            case WOBBLE_RAISE:
                System.out.println("WOBBLE_raiseInState");
                if(!robot.wobble.isWobbleUp(wobblePos))
                {
                    wobbleWait.reset();
                    robot.wobble.raiseWobbleFromFront();
                    wobbleState = WobbleState.WOBBLE_RAISEWAIT;
                    System.out.println("WOBBLE_raise");
                }
                break;
            case WOBBLE_RAISEWAIT:
                System.out.println("WOBBLE_raiseWaitInState");
                if(wobbleWait.time() >= 25)
                {
                    wobbleState = WobbleState.WOBBLE_RAISE;
                    System.out.println("WOBBLE_raiseWait");
                }
                break;
            case WOBBLE_LOWER:
                if(!robot.wobble.isWobbleDown(wobblePos))
                {
                    wobbleWait.reset();
                    wobbleWait.startTime();
                    robot.wobble.lowerWobbleFromFront();
                    wobbleState = WobbleState.WOBBLE_LOWERWAIT;
                }
                break;
            case WOBBLE_LOWERWAIT:
                if(wobbleWait.time() >= 25)
                {
                    wobbleState = WobbleState.WOBBLE_LOWER;
                }
                break;
            case WOBBLE_OFF:
                if (robot.wobble.on) {
                    if (robot.wobble.isWobbleDown(40)) {
                        robot.wobble.wobbleOff();
                    }
                }
                break;
        }

    }

    public void turnTo(double targetAngle, boolean isInputRadians)
    {
        double currentHeading = robot.drive.getPoseEstimate().getHeading();

        if(!isInputRadians)
        {
           targetAngle = Math.toRadians(targetAngle);
        }

        double normAngle = Angle.normDelta(targetAngle - currentHeading);
       /* if (targetAngle < 0)
        {
            normAngle = targetAngle + (Math.PI * 2);
        }*/

        System.out.println("SHOOTER_targetAngle (in Degrees) " + Math.toDegrees(normAngle));

       // double diff= normAngle-currentHeading;
        System.out.println("SHOOTER_Turnto Current  " + Math.toDegrees(currentHeading));
        System.out.println("SHOOTER_Turnto Target  " + Math.toDegrees(targetAngle));
        System.out.println("SHOOTER_Turnto Turn  " + Math.toDegrees(normAngle));
        robot.drive.turnAsync(normAngle);
        //robot.drive.turnAsync(targetAngle);
    }
    public void turnTo(double targetAngle)
    {
        turnTo(targetAngle, true);
    }

    public void powerTurn()
    {
       // PowerTarget=PowerTarget - (isRed*6);
        //turnTo(isRed*-6);

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
        Pose2d CurrentP = robot.drive.getPoseEstimate();
        switch (ringPosition)
        {
            case 0: //A
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        .lineToLinearHeading(new Pose2d(12, isRed * -36, Math.toRadians(70)))
                        .build();
                misswobble = robot.drive.trajectoryBuilder(trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .lineTo(new Vector2d(12, -24*isRed))
                        .build();
                break;
            case 1: //B
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        .lineToLinearHeading(new Pose2d(36, isRed * -12, Math.toRadians(70)))
                        .build();
                misswobble = robot.drive.trajectoryBuilder(trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .lineTo(new Vector2d(36, -24*isRed))
                                .build();
                break;
            case 2: //C
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        .lineToLinearHeading(new Pose2d(60, isRed * -36, Math.toRadians(70)))
                        .build();
                misswobble = robot.drive.trajectoryBuilder(trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .lineTo(new Vector2d(60, -24*isRed))
                        .build();
                break;
        }




        trajectory3 = robot.drive.trajectoryBuilder(misswobble.end(),true)
                //.splineTo(new Vector2d(-55, isRed * -55), 0)
                .splineToLinearHeading(new Pose2d(-30, -48*isRed, Math.toRadians(0)),0)
                .build();

        switch (ringPosition)
        {
            case 0: //A
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end())
                        .splineToLinearHeading(new Pose2d(12, isRed * -36, Math.toRadians(90)), 0)
                        .build();
                break;
            case 1: //B
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end())
                        .splineToLinearHeading(new Pose2d(36, isRed * -12, Math.toRadians(90)), 0)
                        .build();
                break;
            case 2: //C
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory4 = robot.drive.trajectoryBuilder(trajectory3.end())
                        .splineToLinearHeading(new Pose2d(60, isRed * -36, Math.toRadians(90)), 0)
                        .build();
                break;
        }


       trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end(),true)
                .splineToLinearHeading(new Pose2d(12, isRed * -24,Math.toRadians(0)), 0)
                .build();
    }


}

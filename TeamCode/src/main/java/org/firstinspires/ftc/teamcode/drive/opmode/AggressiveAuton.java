package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.Robot;

import Unused.trclib.trclib.TrcPixyCam2;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(group = "drive")
public class AggressiveAuton extends OpMode {
    private boolean debug=false;
    public static double DISTANCE = 60; // in
    double PowerTarget;
    int isRed; // BLUE = -1; RED = 1
    int shootCount = 0;
    int secondShootTotal =0;
    double targetVelocity = 1900;
    boolean shooterReady;
    public Robot robot;
    int ringPosition;
    int wobblePos = 0;
    /*
    private static final double POWEROFFSET = Math.toRadians(6.5);
    private static final double POWEROFFSET2 = Math.toRadians(6);
    private static final double POWEROFFSET3 = Math.toRadians(5);
    */
    private static final double POWEROFFSET = 0;
    private static final double POWEROFFSET2 = 0;
    private static final double POWEROFFSET3 = 0;


    Trajectory trajectory1,trajectory2,trajectory3,pickUpRing, trajectory4,trajectory5,misswobble,strafe1,strafe2;
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
        SHOOT_SECOND_BATCH,
        MISS_WOBBLE,
        PICKUP_RING,
        GRAB_WOBBLE_2,
        BACKUP,
        SHOOTAGAIN,
        RELOADAGAIN,
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
        robot = new Robot(hardwareMap, telemetry,false);
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class so must be called after robot init first time
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        //TODO: Create Pre-Match Selection Using Joystick
        startPos = StartPosEnum.INSIDE_RED;


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


        PoseStorage.currentPose = robot.drive.getPoseEstimate();
        currentState= State.TRAJECTORY_1;

        robot.wobble.closeClaw();
    }

    public void init_loop(){
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        ringPosition = robot.pixy.getStackHeight();
        telemetry.addData("Stack Height", ringPosition);
        telemetry.addData("key", robot.pixy.sensorHeight);
        telemetry.addData("key", robot.pixy.oneRing);
       // telemetry.addData("gyro", Math.toDegrees(Localizer.gyro.readGyro()));
        telemetry.update();
    }

    public void start(){
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        ringPosition = robot.pixy.getStackHeight();
        telemetry.addData("Stack Height", ringPosition);
        telemetry.addData("key", robot.pixy.sensorHeight);
        telemetry.addData("key", robot.pixy.oneRing);
        telemetry.update();
        if (ringPosition==0) {
            trajectory1 = robot.drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), Math.toRadians(-15))
                    //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
                    .build();
            robot.shooter.currentTarget=robot.shooter.redPowerShot1; //affects lift height - change if we shoot for goal
        } else if (ringPosition==1)
        {
            trajectory1 = robot.drive.trajectoryBuilder(startPose)
                  //  .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), Math.toRadians(-15))
                    .splineToLinearHeading(new Pose2d(-7, isRed * -24.5, 0), Math.toRadians(-15))
                    //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
                    .build();
            robot.shooter.currentTarget=robot.shooter.redPowerShot1; //affects lift height - change if we shoot for goal

        } else {
            trajectory1 = robot.drive.trajectoryBuilder(startPose)
                    //TODO Change shooting position?
                        .splineTo(new Vector2d(-24.0,-20.0),Math.toRadians(-15.0))
                        .splineToSplineHeading(new Pose2d(-7.0,-30.0,robot.shooter.angleToGoal(-7, -30, robot.shooter.redGoal)-POWEROFFSET),Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redGoal)-POWEROFFSET), Math.toRadians(-15))
                    //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
                    .build();
            robot.shooter.currentTarget=robot.shooter.redGoal; //affects lift height - change if we shoot for goal

        }
        robot.shooter.update(robot.drive.getPoseEstimate());
        robot.drive.followTrajectoryAsync(trajectory1);
        wobblePos = 450;
        wobbleState = WobbleState.WOBBLE_RAISE;
    }

    public void loop() {
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        robot.drive.update();
        switch (currentState) {
            //TODO: CHANGE ORDER FOR OUTSIDE RED
            case TRAJECTORY_1:
                if (!robot.shooter.isShooterOn) {
                    targetVelocity = robot.shooter.shooterOn();
                }
                if (!robot.drive.isBusy()) {
                    if (debug) {
                        System.out.println("SHOOTER_FIRSTTURN_X " + robot.drive.getPoseEstimate().getX());
                        System.out.println("SHOOTER_FIRSTTURN_Y " + robot.drive.getPoseEstimate().getY());
                    }
                    //turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1) - POWEROFFSET);
                    turnTo(0);
                    currentState = State.FIRST_TURN;
                    if (debug) {
                        System.out.println("SHOOTER_firstAngle " + Math.toDegrees(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1)));
                    }
                }
                break;

            case FIRST_TURN:
                if (!robot.drive.isBusy()){
                    currentState = State.SHOOTER_ON;
                }
                break;

            case SHOOTER_ON:
               // System.out.println("SHOOTER_shooterOn");

                if (robot.shooter.isShooterReady(targetVelocity)) {
                    waitTimer1.reset();
                    currentState = State.SHOOT;
                } //will need to add a timer later to move on in case we never get up to speed
                break;

            case SHOOT:
                boolean done;
                /*if (debug) {
               // System.out.println("SHOOTER_shootInState");
               // System.out.println("SHOOTER_shootInState still turning " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
               }
                 */
                if (!robot.drive.isBusy()&& waitTimer1.time()>500) {//making sure our turn is done  && waitTimer1.time()>1500
                   /* if (isRed==1) {
                        done=robot.drive.getPoseEstimate().getHeading() <= Math.toRadians(PowerTarget);
                    } else {
                        done=robot.drive.getPoseEstimate().getHeading()>= Math.toRadians(PowerTarget);
                    }
                    if (done) {*/
                    /*if (debug) {
                    System.out.println("SHOOTER_now shooting heading " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                    }
                     */
                    if(shootCount<3)
                    {
                        if (debug) {
                            System.out.println("SHOOTER_shoot " + shootCount);
                            System.out.println("SHOOTER_TURN_Final heading" + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                            //   System.out.println("SHOOT_Shooter Ready " + robot.shooter.shooter.getVelocity());
                            //   System.out.println("SHOOT_Shooter Ready " + robot.shooter.isShooterReady(targetVelocity));
                        }
                        robot.shooter.pusherIn();
                        shootCount += 1;
                        waitTimer1.reset();
                    }

                    if (shootCount < 3) {
                        //System.out.println("SHOOTER_shootToTurn");

                        currentState = State.TURN;
                    }
                    else
                    {
                        if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 500) {
                            robot.shooter.shooterOff();
                            robot.shooter.pusherOut();
                            currentState = State.DRIVE_WOBBLE_1;
                        }
                    }
                }
                break;

            case TURN:
               // System.out.println("SHOOTER_turnInState");
                if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 500 ) {
                    if (debug) {
                        System.out.println("SHOOTER_ringShot");
                    }
                    robot.shooter.pusherOut();
                    if (ringPosition<2) powerTurn();
                    currentState = State.SHOOTER_ON;
                }
                break;

            case DRIVE_WOBBLE_1:
                //wobblePos=630;
                setDriveWobble1();
                robot.drive.followTrajectoryAsync(trajectory2);
                wobblePos=650;
                currentState = State.DROP_WOBBLE_1;
            case DROP_WOBBLE_1:
                if(!robot.drive.isBusy()) {
                    wobblePos=800;
                    if(robot.wobble.isWobbleThere(wobblePos))
                    {
                     //   System.out.println("WOBBLE_POS " + robot.wobble.wobble.getCurrentPosition());
                        robot.wobble.openClaw();
                        waitTimer1.reset();
                        currentState = State.MISS_WOBBLE;
                    }
                }
                break;

            case MISS_WOBBLE:
                if(waitTimer1.time()>500)
                {
                    //wobbleState = WobbleState.WOBBLE_LOWER;
                    wobblePos = 700;
                    //robot.drive.followTrajectoryAsync(misswobble);
                    //waitTimer1.reset();
                    robot.drive.followTrajectoryAsync(trajectory3);
                    currentState = State.GRAB_WOBBLE_2;
                }
                break;

            case GRAB_WOBBLE_2:
                wobblePos = 900;
                if (!robot.drive.isBusy()) {
                    robot.wobble.closeClaw();
                    waitTimer1.reset();
                    if(ringPosition > 0 )
                    {
                        currentState = State.BACKUP;

                    }
                    else
                    {
                        currentState = State.GRAB_WAIT;
                    }
                }
                break;

            case BACKUP:
                if (waitTimer1.time()>250) {
                    //robot.intake.turnOn();
                    wobblePos = 450;
                    robot.drive.followTrajectoryAsync(pickUpRing);
                    currentState = State.PICKUP_RING;
                 }
                break;

            case PICKUP_RING:
                if (!robot.drive.isBusy()) {
                    robot.intake.turnOn();
                    robot.drive.followTrajectoryAsync(misswobble);
                    shootCount=0;
                    robot.shooter.currentTarget=robot.shooter.redGoal;
                    if(ringPosition ==1 )
                    {
                        secondShootTotal =1;
                    }
                    else
                    {
                        secondShootTotal =3;
                    }
                    currentState = State.SHOOT_SECOND_BATCH;
                }
                break;

            case SHOOT_SECOND_BATCH:

                // System.out.println("SHOOTER_shooterSecond");
                if (!robot.drive.isBusy()) {
                    robot.intake.turnOff();

                    if (robot.shooter.isShooterReady(targetVelocity)) {
                        waitTimer1.reset();
                        currentState = State.SHOOTAGAIN;
                    } //will need to add a timer later to move on in case we never get up to speed
                }
                break;


            case SHOOTAGAIN:
               // boolean done;
                /*if (debug) {
               // System.out.println("SHOOTER_shootInState");
               // System.out.println("SHOOTER_shootInState still turning " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
               }
                 */
                if (!robot.drive.isBusy()&& waitTimer1.time()>500) {//making sure our turn is done  && waitTimer1.time()>1500
                   /* if (isRed==1) {
                        done=robot.drive.getPoseEstimate().getHeading() <= Math.toRadians(PowerTarget);
                    } else {
                        done=robot.drive.getPoseEstimate().getHeading()>= Math.toRadians(PowerTarget);
                    }
                    if (done) {*/
                    /*if (debug) {
                    System.out.println("SHOOTER_now shooting heading " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                    }
                     */
                    if(shootCount<secondShootTotal)
                    {
                        if (debug) {
                            System.out.println("SHOOTER_shoot " + shootCount);
                            System.out.println("SHOOTER_TURN_Final heading" + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                            //   System.out.println("SHOOT_Shooter Ready " + robot.shooter.shooter.getVelocity());
                            //   System.out.println("SHOOT_Shooter Ready " + robot.shooter.isShooterReady(targetVelocity));
                        }
                        robot.shooter.pusherIn();
                        shootCount += 1;
                        waitTimer1.reset();
                    }

                    if (shootCount < secondShootTotal) {
                        //System.out.println("SHOOTER_shootToTurn");
                        currentState = State.RELOADAGAIN;
                    }
                    else
                    {
                        if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 500) {
                            robot.shooter.shooterOff();
                            robot.shooter.pusherOut();
                            currentState = State.GRAB_WAIT;
                        }
                    }
                }
                break;

            case RELOADAGAIN:
                // System.out.println("SHOOTER_turnInState");
                if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 500 ) {
                    if (debug) {
                        System.out.println("SHOOTER_ringShot");
                    }
                    robot.shooter.pusherOut();
                    currentState = State.SHOOT_SECOND_BATCH;
                }
                break;





            case GRAB_WAIT:
                if (waitTimer1.time()>250 && !robot.drive.isBusy()) {

                    robot.drive.followTrajectoryAsync(trajectory4);
                    //wobbleState = WobbleState.WOBBLE_LOWER;
                    wobblePos = 450;
                    currentState = State.DROP_WOBBLE_2;
                    waitTimer1.reset();
                }
                break;

            case DRIVE_WOBBLE_2:
                if (waitTimer1.time()>2250) {
                    //wobbleState = WobbleState.WOBBLE_RAISE;
                    wobblePos = 630;
                    currentState = State.DROP_WOBBLE_2;
                }
                break;
            case DROP_WOBBLE_2:

                if(!robot.drive.isBusy()) {
                   /* if(ringPosition >0)
                    {
                        robot.intake.turnOff();
                    }*/
                    wobblePos = 850;
                    robot.wobble.openClaw();
                    currentState = State.CLAW_WAIT;
                    waitTimer1.reset();
                }
                break;
            case CLAW_WAIT:
                if (waitTimer1.milliseconds() > 500){
                    currentState = State.PARK;
                    wobblePos = 450;
                    waitTimer1.reset();
                }

                break;

            case PARK:
                //wobbleState = WobbleState.WOBBLE_LOWER;
                if (waitTimer1.milliseconds() > 500) {
                    wobblePos = 0;
                    robot.drive.followTrajectoryAsync(trajectory5);
                    currentState = State.OFF;
                }
                break;
            case OFF:
                if(robot.wobble.isWobbleThere(wobblePos))
                {
                    wobbleState = WobbleState.WOBBLE_OFF;
                }
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
               // System.out.println("WOBBLE_raiseInState");
                //if(!robot.wobble.isWobbleUp(wobblePos))
                robot.wobble.wobbleSetRaise(wobblePos);
                if(!robot.wobble.isWobbleThere(wobblePos))
                {
                    wobbleWait.reset();
                  //  robot.wobble.raiseWobbleFromFront();
                    robot.wobble.wobbleMovetoPosition(wobblePos);
                    wobbleState = WobbleState.WOBBLE_RAISEWAIT;
                   // System.out.println("WOBBLE_raise " + wobblePos);
                }

                break;
            case WOBBLE_RAISEWAIT:
                //System.out.println("WOBBLE_raiseWaitInState");
                if(wobbleWait.time() >= 20)
                {
                    wobbleState = WobbleState.WOBBLE_RAISE;
                    //System.out.println("WOBBLE_raiseWait");
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
        // double diff= normAngle-currentHeading;
        if (debug) {
            System.out.println("SHOOTER_targetAngle (in Degrees) " + Math.toDegrees(normAngle));
            System.out.println("SHOOTER_TURN_X " + robot.drive.getPoseEstimate().getX());
            System.out.println("SHOOTER_TURN_Y " + robot.drive.getPoseEstimate().getY());
            System.out.println("SHOOTER_TURNto Current  " + Math.toDegrees(currentHeading));
            System.out.println("SHOOTER_TURNto Target  " + Math.toDegrees(targetAngle));
            System.out.println("SHOOTER_TURNto Turn  " + Math.toDegrees(normAngle));
        }
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
                strafe1 = robot.drive.trajectoryBuilder(trajectory1.end())
                        .strafeLeft(7.5)
                        .build();
                robot.drive.followTrajectoryAsync(strafe1);

                //turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot2)-POWEROFFSET2);
            }
            else
            {
                strafe2 = robot.drive.trajectoryBuilder(strafe1.end())
                        .strafeLeft(7.5)
                        .build();
                robot.drive.followTrajectoryAsync(strafe2);
                //turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot3)-POWEROFFSET3);
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
                        .lineToLinearHeading(new Pose2d(10, isRed * -44, Math.toRadians(100)))
                        .build();
                trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        .splineToLinearHeading(new Pose2d(-8, -48 * isRed, Math.toRadians(0)),0)
                        .build();
                misswobble = robot.drive.trajectoryBuilder(trajectory3.end(),true)
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .splineToLinearHeading(new Pose2d(-32, -47 * isRed,Math.toRadians(0)),0)
                        .build();

                trajectory4 = robot.drive.trajectoryBuilder(misswobble.end())
                        .splineToLinearHeading(new Pose2d(7.5, isRed * -40, Math.toRadians(90)), 0)
                        .build();
                trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end())
                        .splineToLinearHeading(new Pose2d(3, isRed * -30,Math.toRadians(90)), 0)
                        .build();
                break;
            case 1: //B
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        .lineToLinearHeading(new Pose2d(20, isRed * -32, Math.toRadians(175)))
                        .build();
                trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        .splineToSplineHeading(new Pose2d(-8, -48 * isRed, Math.toRadians(0)),Math.toRadians(180))
                        .addDisplacementMarker(() -> {
                            wobblePos = 900;
                        })
                        .splineToLinearHeading(new Pose2d(-34, -47 * isRed,Math.toRadians(0)),Math.toRadians(180))
                        .build();
               /* misswobble = robot.drive.trajectoryBuilder(trajectory3.end(),true)
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .splineToLinearHeading(new Pose2d(-32, -47 * isRed,Math.toRadians(0)),0)
                        .build();

                */

                pickUpRing = robot.drive.trajectoryBuilder(trajectory3.end())
                        .lineToLinearHeading(new Pose2d(-44, isRed * -55, Math.toRadians(45)))
                        .build();
                misswobble = robot.drive.trajectoryBuilder(pickUpRing.end())
                 .splineToSplineHeading(new Pose2d(-12.0, -30.0, Math.toRadians(40.0)),Math.toRadians(45.0))
                        .addDisplacementMarker(() -> {
                            if (!robot.shooter.isShooterOn) {
                                targetVelocity = robot.shooter.shooterOn();
                            }
                        })
                    .splineToSplineHeading(new Pose2d(-9.0, -22.0, robot.shooter.angleToGoal(-9, -22, robot.shooter.redGoal)),Math.toRadians(90.0))
                        .build();

                trajectory4 = robot.drive.trajectoryBuilder(misswobble.end())
                        .splineToLinearHeading(new Pose2d(28, isRed * -56, Math.toRadians(260)), 0)
                        .addDisplacementMarker(20, () -> {
                            wobblePos = 630;
                        })
                        .addDisplacementMarker(new Vector2d(-8,-22).distTo(new Vector2d(28,-56))*.8, () -> {
                            wobblePos=850;
                        })
                        .build();
                trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end())
                        .splineToLinearHeading(new Pose2d(12, isRed * -55, Math.toRadians(90)), 0)
                        .build();
                break;
            case 2: //C
                //TODO: Verify Wobble Goal Position and Ring Height Map
                trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        .lineToLinearHeading(new Pose2d(48, isRed * -45, Math.toRadians(110)))
                        .build();
                trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        //.splineToLinearHeading(new Pose2d(10, -51 * isRed, Math.toRadians(0)),0)
                        //.splineToLinearHeading(new Pose2d(0, -52 * isRed, Math.toRadians(0)),0)
                        .lineToLinearHeading(new Pose2d(-8, -50 * isRed, Math.toRadians(0)))
                        .build();
                misswobble = robot.drive.trajectoryBuilder(trajectory3.end(),true)
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .splineToLinearHeading(new Pose2d(-32, -48 * isRed,Math.toRadians(0)),0)
                        .build();

                pickUpRing = robot.drive.trajectoryBuilder(misswobble.end())
                        .lineToLinearHeading(new Pose2d(-25, isRed * -50, Math.toRadians(355)))
                        .build();
                trajectory4 = robot.drive.trajectoryBuilder(pickUpRing.end())
                        .lineToLinearHeading(new Pose2d(36, isRed * -54, Math.toRadians(165)))
                        .build();
                trajectory5 = robot.drive.trajectoryBuilder(trajectory4.end())
                        .splineToLinearHeading(new Pose2d(12, isRed * -46,Math.toRadians(90)), 0)
                        .build();
                break;
        }



    }


}

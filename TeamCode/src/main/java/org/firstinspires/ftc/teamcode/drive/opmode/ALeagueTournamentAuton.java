package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Shooter;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class ALeagueTournamentAuton extends OpMode {
    private boolean debug=false;
    public static double DISTANCE = 60; // in
    double PowerTarget;
    double ringTurn;
    int isRed; // BLUE = -1; RED = 1

  //  int secondShootTotal =0;
    //double targetVelocity = 1900;
    boolean shooterReady;
  //  double shootX;
 //   double shootY;
    public Robot robot;
    //int ringPosition;

    /*
    private static final double POWEROFFSET = Math.toRadians(6.5);
    private static final double POWEROFFSET2 = Math.toRadians(6);
    private static final double POWEROFFSET3 = Math.toRadians(5);
    */
    private static final double POWEROFFSET = Math.toRadians(3)*-1;
    private static final double POWEROFFSET2 = 0;
    private static final double POWEROFFSET3 = 0;


   // Trajectory trajectory1,trajectory2,trajectory3,pickUpRing, trajectory4,trajectory5,misswobble,strafe1,strafe2,ring2,ring3,drop1;

    ElapsedTime waitTimer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);





    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        FIRST_TURN,
        SHOOTER_ON,
        SHOOT,
        TURN,
        TURN2,
        RING_TURN,
        DRIVE_WOBBLE_1,
        RING2,
        RING3,
        DONE_RING,
        DROP_WOBBLE_1,
        SHOOT_SECOND_BATCH,
        MISS_WOBBLE,
        PICK_UP_RING,
        PICK_UP_RING_WAIT,
        GO_SHOOT,
        GRAB_WOBBLE_2,
        RELOAD_WAIT,
        WAIT_MORE,
        CHECK_SHOOTER,
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
    AutonWobble autonWobble;
    AutonShooting autonShooting;
    AutonPath autonPath;
    boolean pathsdone = false;
    State currentState;
    @Override

    public void init(){
        robot = new Robot(hardwareMap, telemetry,true);
        autonWobble=new AutonWobble(robot);
        autonPath = new AutonPath(robot);
        autonShooting=new AutonShooting(robot,autonPath);
        robot.shooter.liftState= Shooter.LiftState.STATIC;
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class so must be called after robot init first time
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        //TODO: Create Pre-Match Selection Using Joystick
        autonPath.setStartPos(AutonPath.StartPosEnum.INSIDE_RED);
        PoseStorage.currentPose = robot.drive.getPoseEstimate();
        isRed=autonPath.isRed;
        PoseStorage.isRed=isRed;

        //autonPath.powershotTurnMode= AutonPath.PowershotTurnMode.TURN;
        autonPath.powershotTurnMode= AutonPath.PowershotTurnMode.STRAFE;

        //Pose2d startOutsidePose = new Pose2d(X, Y * isRed, 0);
        setPaths();
        currentState= State.TRAJECTORY_1;


    }

    public void init_loop(){
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        autonPath.ringPosition = robot.pixy.getStackHeight();
        telemetry.addData("Stack Height", autonPath.ringPosition);
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
        autonPath.ringPosition = robot.pixy.getStackHeight();
        telemetry.addData("Stack Height", autonPath.ringPosition);
        telemetry.addData("Pixy", robot.pixy.sensorHeight);
        telemetry.addData("One Ring", robot.pixy.oneRing);
        telemetry.update();
        autonPath.setCurrentTarget(autonPath.currentTargetArray[autonPath.ringPosition]);
     //   robot.shooter.update(robot.drive.getPoseEstimate());//don't need yet
        robot.drive.followTrajectoryAsync(autonPath.trajectory1Array[autonPath.ringPosition]);
        autonWobble.setWobblePos(350);//450

    }

    public void loop() {
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        System.out.println("Loop: " + currentState );
        robot.drive.update();
        switch (currentState) {
            //TODO: CHANGE ORDER FOR OUTSIDE RED
            case TRAJECTORY_1:
                if (!robot.shooter.isShooterOn) {
                    if (autonPath.currentTarget == AutonPath.CurrentTarget.RED_POWERSHOT) {
                        autonShooting.desiredVelocity=1850;
                    }
                        else {
                        autonShooting.desiredVelocity = 2000;
                    }
                        autonShooting.shooterOn();
                }
                if (!robot.drive.isBusy()) {
                    if (debug) {
                        System.out.println("SHOOTER_FIRSTTURN_X " + robot.drive.getPoseEstimate().getX());
                        System.out.println("SHOOTER_FIRSTTURN_Y " + robot.drive.getPoseEstimate().getY());
                    }
                    //turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1) - POWEROFFSET);
                  autonPath.aimFirst(autonPath.firstAngleArray[autonPath.ringPosition],autonPath.currentTargetArray[autonPath.ringPosition]);
                    currentState = State.FIRST_TURN;

                                    }
                break;

            case FIRST_TURN:
                waitTimer1.reset();
                if (!robot.drive.isBusy()){
                    if (autonPath.ringPosition==2) currentState= State.RING_TURN;
                        else currentState= State.DRIVE_WOBBLE_1;
                    autonShooting.shootingState = AutonShooting.ShootingState.SHOOTER_ON;

                }
                break;

            case RING_TURN:
                if (!autonShooting.isBusy()) {
                    autonPath.turnTo(ringTurn);
                    currentState= State.DRIVE_WOBBLE_1;
                }
                break;

            case DRIVE_WOBBLE_1:
                //wobblePos=630;
                if (!robot.drive.isBusy()&&!autonShooting.isBusy()) {
                    //setDriveWobble1();
                    robot.drive.followTrajectoryAsync(autonPath.trajectory2Array[autonPath.ringPosition]);
                    if (autonPath.ringPosition ==2) robot.intake.turnOn();
                    //wobblePos=650;now in trajectory

                    if (autonPath.ringPosition < 2) currentState = State.DROP_WOBBLE_1;
                    else currentState = State.RING2;//DONE_RING;//State.RING2;
                }
                break;
            case RING2:
                if(!robot.drive.isBusy()) {
                    robot.drive.followTrajectoryAsync(autonPath.ring2Array[autonPath.ringPosition]);
                    currentState= State.DONE_RING;//RING3;
                }
                break;
            case RING3:
                if(!robot.drive.isBusy()) {
                    robot.drive.followTrajectoryAsync(autonPath.ring3Array[autonPath.ringPosition]);
                    currentState= State.DONE_RING;
                }
                break;
            case DONE_RING:
                if(!robot.drive.isBusy()) {

                    robot.drive.followTrajectoryAsync(autonPath.drop1Array[autonPath.ringPosition]);
                    currentState= State.DROP_WOBBLE_1;
                }
                break;
            case DROP_WOBBLE_1:
                autonWobble.setWobblePos(630);
                if(!robot.drive.isBusy()) {
                  //  if (autonPath.ringPosition == 2) robot.intake.turnOn();
                    autonWobble.setWobblePos(800);
                    if(!autonWobble.isBusy())
                    {
                     //   System.out.println("WOBBLE_POS " + robot.wobble.wobble.getCurrentPosition());
                        autonWobble.openClaw();
                        waitTimer1.reset();
                        currentState = State.GRAB_WOBBLE_2;
                    }
                }
                break;

            case GRAB_WOBBLE_2:
                if(waitTimer1.time()>400)
                {
                    //wobbleState = WobbleState.WOBBLE_LOWER;
                    //robot.drive.followTrajectoryAsync(misswobble);
                    //waitTimer1.reset();
                    autonWobble.setWobblePos(700);
                    robot.drive.followTrajectoryAsync(autonPath.trajectory3Array[autonPath.ringPosition]);
                    //wobblePos = 700; now in trajectory3
                    currentState = State.MISS_WOBBLE;
                }
                break;

            case MISS_WOBBLE:
                //wobblePos = 900; now in trajectory3
                if (!robot.drive.isBusy()) {
                    autonWobble.closeClaw();
                    waitTimer1.reset();
                    if(autonPath.ringPosition == 2 )
                    {
                        currentState = State.RELOAD_WAIT;
                    }
                    else if (autonPath.ringPosition == 1 )
                    {
                        currentState = State.PICK_UP_RING;
                    }
                    else
                    {
                        currentState = State.GRAB_WAIT;
                    }
                }
                break;

            case PICK_UP_RING:
                if(!robot.drive.isBusy()) {
                    if (waitTimer1.milliseconds()>250) {
                        autonWobble.setWobblePos(450);
                        robot.drive.followTrajectoryAsync(autonPath.pickUpRingArray[autonPath.ringPosition]);
                        robot.intake.turnOn();
                        currentState = State.RELOAD_WAIT;
                    }
                }
                break;

            case RELOAD_WAIT:
                autonShooting.shooterOn();
                if(!robot.drive.isBusy()) {
                    if (waitTimer1.milliseconds() > 250) {
                        //robot.intake.turnOn();
                        robot.drive.followTrajectoryAsync(autonPath.misswobbleArray[autonPath.ringPosition]);
                        autonWobble.setWobblePos(450);
                        //robot.drive.followTrajectoryAsync(pickUpRing);
                        autonShooting.desiredVelocity = 0;
                        currentState = State.CHECK_SHOOTER;



                    }
                }
                break;
            case CHECK_SHOOTER:
                if (autonPath.ringPosition==1) {
                    if (waitTimer1.milliseconds() > 6500) {
                        autonShooting.shooterOn();
                    }
                    if (waitTimer1.milliseconds() > 7500) {
                        robot.intake.turnOff();
                        if (!robot.drive.isBusy()) {
                            autonPath.turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redGoal)-POWEROFFSET);
                            autonShooting.shootingState = AutonShooting.ShootingState.GO_SHOOT;
                            currentState = State.GRAB_WAIT;
                        }
                    }
                } else {
                    robot.intake.turnOff();
                    if (!robot.drive.isBusy()) {
                       autonPath.turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redGoal)-POWEROFFSET);
                        autonShooting.shootingState = AutonShooting.ShootingState.GO_SHOOT;
                        currentState = State.GRAB_WAIT;
                    }
                }

                break;



            case GRAB_WAIT:
                if (waitTimer1.time()>250 && !robot.drive.isBusy()&&!autonShooting.isBusy()) {

                    robot.drive.followTrajectoryAsync(autonPath.trajectory4Array[autonPath.ringPosition]);
                    //wobbleState = WobbleState.WOBBLE_LOWER;
                    autonWobble.setWobblePos(450);
                    currentState = State.DROP_WOBBLE_2;
                    waitTimer1.reset();
                }
                break;

            case DRIVE_WOBBLE_2:
                if (waitTimer1.time()>750) {
                    //wobbleState = WobbleState.WOBBLE_RAISE;
                    autonWobble.setWobblePos(550);//630;
                    currentState = State.DROP_WOBBLE_2;
                }
                break;
            case DROP_WOBBLE_2:

                if(!robot.drive.isBusy()) {
                   /* if(ringPosition >0)
                    {
                        robot.intake.turnOff();
                    }*/
                    autonWobble.setWobblePos(800);
                   // wobblePos = 850;
                    if(!autonWobble.isBusy()) {
                        autonWobble.openClaw();
                        currentState = State.CLAW_WAIT;
                        waitTimer1.reset();
                    }
                }
                break;
            case CLAW_WAIT:
                if (waitTimer1.milliseconds() > 1){
                    currentState = State.PARK;

                    waitTimer1.reset();
                }

                break;

            case PARK:
                //wobbleState = WobbleState.WOBBLE_LOWER;
                if (waitTimer1.milliseconds() > 400) {
                    autonWobble.setWobblePos(0);
                    robot.drive.followTrajectoryAsync(autonPath.trajectory5Array[autonPath.ringPosition]);

                    currentState = State.OFF;
                }
                break;
            case OFF:

              /*  if(robot.wobble.isWobbleThere(wobblePos))
                {
                    wobbleState = WobbleState.WOBBLE_OFF;
                }*/
                break;
        }
        autonWobble.handleWobble();

        autonShooting.handleShooting();

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

            /*
            telemetry.addData("Stack Height", ringPosition);
            telemetry.addData("key", robot.pixy.sensorHeight);
            telemetry.addData("key", robot.pixy.oneRing);
            telemetry.update();*/
    }




    public void setDriveWobble1()
    {
        //Pose2d CurrentP = robot.drive.getPoseEstimate();

             //A
                //TODO: Verify Wobble Goal Position and Ring Height Map
                 autonPath.trajectory2Array[0] = robot.drive.trajectoryBuilder(autonPath.strafe2Array[0].end())
             //   autonPath.trajectory2Array[0] = robot.drive.trajectoryBuilder(getCurrentP(autonPath.firstShotArray[0],autonPath.firstAngleArray[0],autonPath.currentTargetArray[0]))
                        .lineToLinearHeading(new Pose2d(8, isRed * -45, Math.toRadians(100)))
                        .addTemporalMarker(1, () -> {
                            autonWobble.setWobblePos(650);
                        })
                        .build();
               /* Revised paths
                trajectory3 = robot.drive.trajectoryBuilder(trajectory2.end())

                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        .splineToLinearHeading(new Pose2d(-8, -48 * isRed, Math.toRadians(0)),0)
                        .build();
                misswobble = robot.drive.trajectoryBuilder(trajectory3.end(),true)
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .splineToLinearHeading(new Pose2d(-32, -47 * isRed,Math.toRadians(0)),0)
                        .build();
*/
                autonPath.trajectory3Array[0] = robot.drive.trajectoryBuilder(autonPath.trajectory2Array[0].end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        .splineToSplineHeading(new Pose2d(-8, -48 * isRed, Math.toRadians(0)),Math.toRadians(180))
                        /*.addTemporalMarker(.5, () -> {
                            wobblePos=700;
                        })

                         */
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(900);
                        })
                        .splineToLinearHeading(new Pose2d(-35, -47 * isRed,Math.toRadians(0)),Math.toRadians(185))//x=-34
                        .build();
                autonPath.trajectory4Array[0] = robot.drive.trajectoryBuilder(autonPath.trajectory3Array[0].end())
                        .splineToLinearHeading(new Pose2d(9.5, isRed * -34, Math.toRadians(90)), 0)
                        .build();
                autonPath.trajectory5Array[0] = robot.drive.trajectoryBuilder(autonPath.trajectory4Array[0].end())
                        .splineToLinearHeading(new Pose2d(3, isRed * -28,Math.toRadians(90)), 0)
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(0);
                        })
                        .build();
            //B
                //TODO: Verify Wobble Goal Position and Ring Height Map
                //TODO: Fix path choice for turn
                autonPath.trajectory2Array[1] = robot.drive.trajectoryBuilder(autonPath.strafe2Array[1].end())
               // autonPath.trajectory2Array[1] = robot.drive.trajectoryBuilder(getCurrentP(autonPath.firstShotArray[1],autonPath.firstAngleArray[1],autonPath.currentTargetArray[1]))
                    /*.splineTo(new Vector2d(-8.0,-50.0),Math.toRadians(0.0))
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(450);
                        })

                     */
                   .lineToSplineHeading(new Pose2d(20.0,-32.0,Math.toRadians(175.0)))
                    //    .lineToSplineHeading(new Pose2d(22.0,-32.0,Math.toRadians(175.0)))
                //.lineToLinearHeading(new Pose2d(20, isRed * -32, Math.toRadians(175)))
                     .build();
                autonPath.trajectory3Array[1] = robot.drive.trajectoryBuilder(autonPath.trajectory2Array[1].end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        .splineToSplineHeading(new Pose2d(-8, -48 * isRed, Math.toRadians(0)),Math.toRadians(180))
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(700);
                        })
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(900);
                        })
                        .splineToLinearHeading(new Pose2d(-35, -47 * isRed,Math.toRadians(0)),Math.toRadians(185))//x=-34
                        .build();
               /* misswobble = robot.drive.trajectoryBuilder(trajectory3.end(),true)
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .splineToLinearHeading(new Pose2d(-32, -47 * isRed,Math.toRadians(0)),0)
                        .build();

                */

                    autonPath.pickUpRingArray[1] = robot.drive.trajectoryBuilder(autonPath.trajectory3Array[1].end())
                        .lineToLinearHeading(new Pose2d(-46, isRed * -55, Math.toRadians(45)))
                        .build();
                autonPath.misswobbleArray[1] = robot.drive.trajectoryBuilder(autonPath.pickUpRingArray[1].end())
               //         .splineToLinearHeading(new Pose2d(-8.0, -36.0, Math.toRadians(0.0)),Math.toRadians(90.0))
                        .splineToSplineHeading(new Pose2d(-12.0, -30.0, Math.toRadians(40.0)),Math.toRadians(45.0))
                        .addDisplacementMarker(() -> {
                            if (!robot.shooter.isShooterOn) {
                                autonShooting.shooterOn();
                            }
                        })
                    .splineToSplineHeading(new Pose2d(-10.0, -36.0, robot.shooter.angleToGoal(-10, -36, robot.shooter.redGoal)),Math.toRadians(90.0))


                        .build();

                autonPath.trajectory4Array[1] = robot.drive.trajectoryBuilder(autonPath.misswobbleArray[1].end())
                        //.splineToLinearHeading(new Pose2d(30, isRed * -56, Math.toRadians(260)), 0)
                        .lineToLinearHeading(new Pose2d(10, isRed * -40, Math.toRadians(180)))
                        /*.addDisplacementMarker(20, () -> {
                            wobblePos = 630;
                        })
                        .addDisplacementMarker(new Vector2d(-8,-36).distTo(new Vector2d(15,-38))*.8, () -> {
                            wobblePos=900;
                        })*/
                        .build();
                autonPath.trajectory5Array[1] = robot.drive.trajectoryBuilder(autonPath.trajectory4Array[1].end())
                        //.splineToLinearHeading(new Pose2d(12, isRed * -52, Math.toRadians(90)), 0)
                        .lineToLinearHeading(new Pose2d(6, isRed * -37, Math.toRadians(180)))
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(0);
                        })
                        .build();

             //C
                //TODO: Verify Wobble Goal Position and Ring Height Map
                MinVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(MAX_ANG_VEL),
                        new MecanumVelocityConstraint(17, TRACK_WIDTH)
                ));
                ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
                Vector2d ringVec =new Vector2d (-24,-36);//(-12,-42) (-15,-40),(-19,-39)//(-15,-43)//(-19,-39)
                //Vector2d nowVec=new Vector2d(CurrentP.getX(),CurrentP.getY());
        Pose2d cur=getCurrentP(autonPath.firstShotArray[2],autonPath.firstAngleArray[2],autonPath.currentTargetArray[2]);
        Vector2d nowVec=new Vector2d(cur.getX(),cur.getY());
        Vector2d difference=ringVec.minus(nowVec);
                double goAngle = difference.angle();
                ringTurn=goAngle;

                autonPath.trajectory2Array[2] = robot.drive.trajectoryBuilder(cur)//getCurrentP(autonPath.firstShotArray[2],autonPath.firstAngleArray[2],autonPath.currentTargetArray[2]))
                        //.lineToLinearHeading(new Pose2d(-26.0,-31.0,Math.toRadians(300.0)))
                        //.lineToLinearHeading(new Pose2d(-26.0,-33.0,Math.toRadians(320.0)))
                        //.lineTo(ringVec,velConstraint,accelConstraint)
                        .lineToLinearHeading(new Pose2d(ringVec,goAngle),velConstraint,accelConstraint)//goAngle was 325
                      //  .splineToSplineHeading(new Pose2d(-12.0,-52.0,Math.toRadians(320)),0, new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(10,DriveConstants.TRACK_WIDTH))),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        /*.lineToSplineHeading(new Pose2d(-28.0,-20.0,Math.toRadians(307)))
                        .splineToSplineHeading(new Pose2d(-12.0,-52.0,Math.toRadians(307)),0, new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(10,DriveConstants.TRACK_WIDTH))),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker(() -> {
                            wobblePos = 650;
                        })
                        .splineToSplineHeading(new Pose2d(48.0,-45.0,Math.toRadians(110.0)),Math.toRadians(320.0))*/

                        .addDisplacementMarker(() -> {
                            robot.intake.turnOff();
                        })

                        .build();
                autonPath.ring2Array[2] = robot.drive.trajectoryBuilder(autonPath.trajectory2Array[2].end())
                        .lineToSplineHeading(new Pose2d(-20,-30.0,Math.toRadians(240.0)))
                        //.lineToLinearHeading(new Pose2d(-24.0,-39.0,Math.toRadians(320.0)))
                       /* .lineToSplineHeading(new Pose2d(-28.0,-20.0,Math.toRadians(307)))
                        .splineToSplineHeading(new Pose2d(-12.0,-52.0,Math.toRadians(307)),0, new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(10,DriveConstants.TRACK_WIDTH))),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker(() -> {
                            wobblePos = 650;
                        })
                        .splineToSplineHeading(new Pose2d(48.0,-45.0,Math.toRadians(110.0)),Math.toRadians(320.0))

                        */

                        .build();

                autonPath.ring3Array[2] = robot.drive.trajectoryBuilder(autonPath.ring2Array[2].end())

                        .lineToLinearHeading(new Pose2d(-20.0,-44.0,Math.toRadians(330.0)))

                      /*  .lineToSplineHeading(new Pose2d(-28.0,-20.0,Math.toRadians(307)))
                        .splineToSplineHeading(new Pose2d(-12.0,-52.0,Math.toRadians(307)),0, new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(10,DriveConstants.TRACK_WIDTH))),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker(() -> {
                            wobblePos = 650;
                        })
                        .splineToSplineHeading(new Pose2d(48.0,-45.0,Math.toRadians(110.0)),Math.toRadians(320.0))
                        */
                        .build();
                MinVelocityConstraint velConstraint2 = new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(MAX_ANG_VEL),
                        new MecanumVelocityConstraint(55, TRACK_WIDTH)
                ));
                ProfileAccelerationConstraint accelConstraint2 = new ProfileAccelerationConstraint(55);
              //  drop1 = robot.drive.trajectoryBuilder(trajectory2.end())
                Vector2d dropVec =new Vector2d(53,-45);//-45//(54,-42)
                autonPath.drop1Array[2] = robot.drive.trajectoryBuilder(autonPath.ring2Array[2].end())
                      //  .splineToSplineHeading(new Pose2d(-20,-43.0,Math.toRadians(270.0)),0)
                        //.splineToSplineHeading(new Pose2d(48.0,-45.0,Math.toRadians(110.0)),Math.toRadians(320.0))
                      //  .lineToSplineHeading(new Pose2d(53.0,-45.0,Math.toRadians(110.0)))

                       // .lineToSplineHeading(new Pose2d(53.0,-45.0,Math.toRadians(110.0)),velConstraint2,accelConstraint2)
                        .lineToSplineHeading(new Pose2d(dropVec,Math.toRadians(110.0)),velConstraint2,accelConstraint2)
                        .addTemporalMarker(.5, () -> {
                            robot.intake.turnOn();
                        })
                            .addSpatialMarker(new Vector2d(dropVec.getX()-10, dropVec.getY()), () -> {
                                autonWobble.setWobblePos(800);
                        })
                        /*.addDisplacementMarker(2,() -> {
                            wobblePos = 650;
                        })*/

                        .build();
/*
                trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        .lineToLinearHeading(new Pose2d(48, isRed * -45, Math.toRadians(110)))
                        .build();
  */
                autonPath.trajectory3Array[2] = robot.drive.trajectoryBuilder(autonPath.drop1Array[2].end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        .splineToSplineHeading(new Pose2d(-8, -46 * isRed, Math.toRadians(0)),Math.toRadians(180))//-46
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(700);
                        })
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(900);
                        })
                        .splineToLinearHeading(new Pose2d(-34, -43 * isRed,Math.toRadians(0)),Math.toRadians(195))//180
                        .build();
                /*
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
*/
                /*pickUpRing = robot.drive.trajectoryBuilder(misswobble.end())
                        .lineToLinearHeading(new Pose2d(-25, isRed * -50, Math.toRadians(355)))
                        .build();

                 */
                autonPath.misswobbleArray[2] = robot.drive.trajectoryBuilder(autonPath.trajectory3Array[2].end())
                        .splineToLinearHeading(new Pose2d(-8.0, -39.0, Math.toRadians(0.0)),Math.toRadians(90.0),velConstraint2,accelConstraint2)
                        /*.splineToSplineHeading(new Pose2d(-12.0, -30.0, Math.toRadians(40.0)),Math.toRadians(45.0))
                        .addDisplacementMarker(() -> {
                            if (!robot.shooter.isShooterOn) {
                                targetVelocity = robot.shooter.shooterOn();
                            }
                        })
                    .splineToSplineHeading(new Pose2d(-9.0, -22.0, robot.shooter.angleToGoal(-9, -22, robot.shooter.redGoal)),Math.toRadians(90.0))

                         */
                        .build();
                Vector2d drop2Vec= new Vector2d(37,-48);//y=-58
                autonPath.trajectory4Array[2] = robot.drive.trajectoryBuilder(autonPath.misswobbleArray[2].end())
                        .lineToLinearHeading(new Pose2d(drop2Vec, Math.toRadians(160)),velConstraint2,accelConstraint2)
                        .addTemporalMarker(2, () -> {
                            autonWobble.setWobblePos(550);
                        })
                        .addSpatialMarker(new Vector2d(drop2Vec.getX()-10, drop2Vec.getY()), () -> {
                            autonWobble.setWobblePos(700);
                        })
                        .build();
                autonPath.trajectory5Array[2] = robot.drive.trajectoryBuilder(autonPath.trajectory4Array[2].end())
                        .lineToLinearHeading(new Pose2d(12, isRed * -52,Math.toRadians(170)), velConstraint2,accelConstraint2)
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(0);
                        })
                        .build();

           }

    public void setStrafe(){
        MinVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(20, TRACK_WIDTH)
        ));//(Math.toRadians(90))
        double head=Math.toRadians(0);//10
        ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(20);
        autonPath.strafe1Array[0] = robot.drive.trajectoryBuilder(autonPath.trajectory1Array[0].end())
                .lineToLinearHeading(new Pose2d(autonPath.firstShotArray[0].getX(), autonPath.firstShotArray[0].getY() + 7.5, head),velConstraint,accelConstraint)
                .build();
        autonPath.strafe2Array[0] = robot.drive.trajectoryBuilder(autonPath.strafe1Array[0].end())
                //.strafeLeft(8.5)
                .lineToLinearHeading(new Pose2d(autonPath.firstShotArray[0].getX(),autonPath.firstShotArray[0].getY()+15,head),velConstraint,accelConstraint)//was 14 for ringposition==0
                .build();
        autonPath.strafe1Array[1] = robot.drive.trajectoryBuilder(autonPath.trajectory1Array[1].end())
                .lineToLinearHeading(new Pose2d(autonPath.firstShotArray[1].getX(), autonPath.firstShotArray[1].getY() + 7.5, head),velConstraint,accelConstraint)
                .build();
        autonPath.strafe2Array[1] = robot.drive.trajectoryBuilder(autonPath.strafe1Array[1].end())
                //.strafeLeft(8.5)
                .lineToLinearHeading(new Pose2d(autonPath.firstShotArray[1].getX(),autonPath.firstShotArray[1].getY()+15,head),velConstraint,accelConstraint)//was 14 for ringposition==0
                .build();
        autonPath.strafe1Array[2] = robot.drive.trajectoryBuilder(autonPath.trajectory1Array[2].end())
                .lineToLinearHeading(new Pose2d(autonPath.firstShotArray[2].getX(), autonPath.firstShotArray[2].getY() + 7.5, head),velConstraint,accelConstraint)
                .build();
        autonPath.strafe2Array[2] = robot.drive.trajectoryBuilder(autonPath.strafe1Array[2].end())
                //.strafeLeft(8.5)
                .lineToLinearHeading(new Pose2d(autonPath.firstShotArray[2].getX(),autonPath.firstShotArray[2].getY()+15,head),velConstraint,accelConstraint)//was 14 for ringposition==0
                .build();

    }
public void setPaths()
    {

            //0
            autonPath.firstShotArray[0]=new Vector2d(-10,isRed*-17);//(-10,-22)
            //autonPath.firstShot=
            //shootX=-10;
            //shootY=-22;
            autonPath.currentTargetArray[0]=AutonPath.CurrentTarget.RED_POWERSHOT;
            autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_POWERSHOT);
            autonPath.trajectory1Array[0]=autonPath.setFirstTrajectory(autonPath.currentTargetArray[0],autonPath.firstShotArray[0]);
            autonPath.firstAngleArray[0]=autonPath.firstAngle;
            /*
            trajectory1 = robot.drive.trajectoryBuilder(autonPath.startPose)
                    //.splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), Math.toRadians(-15))
                    .lineToLinearHeading(new Pose2d(autonPath.firstShot, 0))
                    .build();
            autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_POWERSHOT); //affects lift height - change if we shoot for goal

             */
       //1

            autonPath.firstShotArray[1]=new Vector2d(-10,isRed*-17);//-45,-22//-10,-23
            autonPath.currentTargetArray[1]=AutonPath.CurrentTarget.RED_POWERSHOT;
        autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_POWERSHOT);
        autonPath.trajectory1Array[1]=autonPath.setFirstTrajectory(autonPath.currentTargetArray[1],autonPath.firstShotArray[1]);
        autonPath.firstAngleArray[1]=autonPath.firstAngle;
            //shootX=-45;
            //shootY=-22;//-20;
            /*
            trajectory1 = robot.drive.trajectoryBuilder(autonPath.startPose)
                  //  .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), Math.toRadians(-15))
                   // .splineToLinearHeading(new Pose2d(-7, isRed * -24.5, 0), Math.toRadians(-15))
                    //.lineToLinearHeading(new Pose2d(shootX, isRed * shootY, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(autonPath.firstShot, robot.shooter.angleToGoal(autonPath.firstShot, robot.shooter.redGoal)-POWEROFFSET))
                    //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
                    .build();
            //robot.shooter.currentTarget=robot.shooter.redPowerShot1; //affects lift height - change if we shoot for goal


             */


            autonPath.firstShotArray[2]=new Vector2d(-45,isRed*-22);
        autonPath.currentTargetArray[2]=AutonPath.CurrentTarget.RED_GOAL;
            autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_GOAL);
        autonPath.trajectory1Array[2]=autonPath.setFirstTrajectory(autonPath.currentTargetArray[2],autonPath.firstShotArray[2]);
        autonPath.firstAngleArray[2]=autonPath.firstAngle;

            //shootX=-45;
            //shootY=-22;
            /*
            trajectory1 = robot.drive.trajectoryBuilder(autonPath.startPose)
                    //TODO Change shooting position?
                    //.lineToLinearHeading(new Pose2d(shootX, isRed * -24, 0))
                    .lineToLinearHeading(new Pose2d(autonPath.firstShot, robot.shooter.angleToGoal(autonPath.firstShot, robot.shooter.redGoal)-POWEROFFSET))
                   */
                    /*prior code
                        .splineTo(new Vector2d(-24.0,-20.0),Math.toRadians(-15.0))
                        .splineToSplineHeading(new Pose2d(-7.0,-30.0,robot.shooter.angleToGoal(-7, -30, robot.shooter.redGoal)-POWEROFFSET),Math.toRadians(0))

                     */
            //.splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redGoal)-POWEROFFSET), Math.toRadians(-15))
            //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
            //.build();

            //robot.shooter.currentTarget=robot.shooter.redPowerShot1; //affects lift height - change if we shoot for goal

            //robot.shooter.liftState= Shooter.LiftState.DYNAMIC;

        setStrafe();
        setDriveWobble1();

    }

    public Pose2d getCurrentP(Vector2d firstVec, double angle,AutonPath.CurrentTarget target){
        //if (autonPath.powershotTurnMode==AutonPath.PowershotTurnMode.TURN) {
        double endAngle=0;
        switch (target){
            case RED_POWERSHOT:

                endAngle=angle+9;
                break;
            case RED_GOAL:
                endAngle=angle;
                break;
        }
        return new Pose2d(firstVec,endAngle);
        /*}
        else {
            return autonPath.strafe2Array[autonPath.ringPosition].end();
        }

         */
    }

}

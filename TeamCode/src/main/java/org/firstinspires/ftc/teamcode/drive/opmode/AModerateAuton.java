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
public class AModerateAuton extends OpMode {
    private boolean debug=false;
    public static double DISTANCE = 60; // in
    double PowerTarget;
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

        autonPath.powershotTurnMode= AutonPath.PowershotTurnMode.TURN;

        //Pose2d startOutsidePose = new Pose2d(X, Y * isRed, 0);

        currentState= State.TRAJECTORY_1;


    }

    public void init_loop(){
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        autonPath.ringPosition = robot.pixy.getStackHeight();
        telemetry.addData("Stack Height", autonPath.ringPosition);
        telemetry.addData("current reading", robot.pixy.sensorHeight);
        telemetry.addData("4 ring calibration", robot.pixy.oneRing);
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
        telemetry.addData("current reading", robot.pixy.sensorHeight);
        telemetry.addData("4 ring calibration", robot.pixy.oneRing);
        telemetry.update();
        if (autonPath.ringPosition==0) {
            autonPath.firstShot=new Vector2d(-10,isRed*-23);
            //shootX=-10;
            //shootY=-22;
            autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_POWERSHOT);

            /*
            trajectory1 = robot.drive.trajectoryBuilder(autonPath.startPose)
                    //.splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), Math.toRadians(-15))
                    .lineToLinearHeading(new Pose2d(autonPath.firstShot, 0))
                    .build();
            autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_POWERSHOT); //affects lift height - change if we shoot for goal

             */
        } else if (autonPath.ringPosition==1)
        {
            autonPath.firstShot=new Vector2d(-10,isRed*-23);//-45,-22
            autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_POWERSHOT);
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

        } else {
            autonPath.firstShot=new Vector2d(-10,isRed*-23);
            autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_POWERSHOT);
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
        }
        autonPath.trajectory1=autonPath.setFirstTrajectory();
     //   robot.shooter.update(robot.drive.getPoseEstimate());//don't need yet
        robot.drive.followTrajectoryAsync(autonPath.trajectory1);
        autonWobble.setWobblePos(450);

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
                  autonPath.aimFirst();
                    currentState = State.FIRST_TURN;

                                    }
                break;

            case FIRST_TURN:
                waitTimer1.reset();
                if (!robot.drive.isBusy()){
                    currentState= State.DRIVE_WOBBLE_1;
                    autonShooting.isBusy=true;
                    autonShooting.shootingState = AutonShooting.ShootingState.SHOOTER_ON;

                }
                break;


            case DRIVE_WOBBLE_1:
                //wobblePos=630;
                if (!autonShooting.isBusy()) {
                    setDriveWobble1();
                    robot.drive.followTrajectoryAsync(autonPath.trajectory2);
                  //  if (autonPath.ringPosition ==2) robot.intake.turnOn();
                    //wobblePos=650;now in trajectory
                    currentState = State.DROP_WOBBLE_1;
                   // if (autonPath.ringPosition < 2) currentState = State.DROP_WOBBLE_1;
                   // else currentState = State.RING2;//DONE_RING;//State.RING2;
                }
                break;
            case RING2:
                if(!robot.drive.isBusy()) {
                    robot.drive.followTrajectoryAsync(autonPath.ring2);
                    currentState= State.DONE_RING;//RING3;
                }
                break;
            case RING3:
                if(!robot.drive.isBusy()) {
                    robot.drive.followTrajectoryAsync(autonPath.ring3);
                    currentState= State.DONE_RING;
                }
                break;
            case DONE_RING:
                if(!robot.drive.isBusy()) {

                    robot.drive.followTrajectoryAsync(autonPath.drop1);
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
                    robot.drive.followTrajectoryAsync(autonPath.trajectory3);
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
                        currentState = State.GRAB_WAIT;
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
                        robot.drive.followTrajectoryAsync(autonPath.pickUpRing);
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
                        robot.drive.followTrajectoryAsync(autonPath.misswobble);
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
                        autonShooting.shootingState = AutonShooting.ShootingState.GO_SHOOT;
                        currentState = State.GRAB_WAIT;
                    }
                } else {
                    robot.intake.turnOff();
                    autonShooting.shootingState = AutonShooting.ShootingState.GO_SHOOT;
                    currentState = State.GRAB_WAIT;
                }
                break;

            case GRAB_WAIT:
                if (waitTimer1.time()>250 && !robot.drive.isBusy()&&!autonShooting.isBusy()) {

                    robot.drive.followTrajectoryAsync(autonPath.trajectory4);
                    //wobbleState = WobbleState.WOBBLE_LOWER;
                    autonWobble.setWobblePos(450);
                    currentState = State.DROP_WOBBLE_2;
                    waitTimer1.reset();
                }
                break;

            case DRIVE_WOBBLE_2:
                if (waitTimer1.time()>750) {
                    //wobbleState = WobbleState.WOBBLE_RAISE;
                    autonWobble.setWobblePos(650);//630;
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
                        //robot.intake.turnOff();
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
                    //autonWobble.setWobblePos(0);
                    robot.drive.followTrajectoryAsync(autonPath.trajectory5);

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
            telemetry.addData("key", robot.pixy.oneRing);*/
            telemetry.update();

    }




    public void setDriveWobble1()
    {
        Pose2d CurrentP = robot.drive.getPoseEstimate();
        switch (autonPath.ringPosition)
        {
            case 0: //A
                //TODO: Verify Wobble Goal Position and Ring Height Map
                autonPath.trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        .lineToLinearHeading(new Pose2d(12, isRed * -46, Math.toRadians(100)))
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
                autonPath.trajectory3 = robot.drive.trajectoryBuilder(autonPath.trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        .splineToSplineHeading(new Pose2d(-8, -48 * isRed, Math.toRadians(0)),Math.toRadians(180))
                        /*.addTemporalMarker(.5, () -> {
                            wobblePos=700;
                        })

                         */
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(900);
                        })
                        .splineToLinearHeading(new Pose2d(-34, -48 * isRed,Math.toRadians(0)),Math.toRadians(180))
                        .build();
                autonPath.trajectory4 = robot.drive.trajectoryBuilder(autonPath.trajectory3.end())
                        .splineToLinearHeading(new Pose2d(9.5, isRed * -36, Math.toRadians(90)), 0)
                        .build();
                autonPath.trajectory5 = robot.drive.trajectoryBuilder(autonPath.trajectory4.end())
                        .splineToLinearHeading(new Pose2d(3, isRed * -28,Math.toRadians(90)), 0)
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(0);
                        })
                        .build();
                break;
            case 1: //B
                //TODO: Verify Wobble Goal Position and Ring Height Map
                autonPath.trajectory2 = robot.drive.trajectoryBuilder(new Pose2d(-10,-23,Math.toRadians(9)))//CurrentP
                    /*.splineTo(new Vector2d(-8.0,-50.0),Math.toRadians(0.0))
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(450);
                        })

                     */
                   .lineToSplineHeading(new Pose2d(20.0,-32.0,Math.toRadians(175.0)))
                    //    .lineToSplineHeading(new Pose2d(22.0,-32.0,Math.toRadians(175.0)))
                //.lineToLinearHeading(new Pose2d(20, isRed * -32, Math.toRadians(175)))
                     .build();
                autonPath.trajectory3 = robot.drive.trajectoryBuilder(autonPath.trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                        .splineToSplineHeading(new Pose2d(-8, -48 * isRed, Math.toRadians(0)),Math.toRadians(180))
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(700);
                        })
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(900);
                        })
                        .splineToLinearHeading(new Pose2d(-34, -46 * isRed,Math.toRadians(0)),Math.toRadians(180))
                        .build();
               /* misswobble = robot.drive.trajectoryBuilder(trajectory3.end(),true)
                        //.splineTo(new Vector2d(-55, isRed * -55), 0)
                        .splineToLinearHeading(new Pose2d(-32, -47 * isRed,Math.toRadians(0)),0)
                        .build();

                */

                    autonPath.pickUpRing = robot.drive.trajectoryBuilder(autonPath.trajectory3.end())
                        .lineToLinearHeading(new Pose2d(-44, isRed * -55, Math.toRadians(45)))
                        .build();
                autonPath.misswobble = robot.drive.trajectoryBuilder(autonPath.pickUpRing.end())
               //         .splineToLinearHeading(new Pose2d(-8.0, -36.0, Math.toRadians(0.0)),Math.toRadians(90.0))
                        .splineToSplineHeading(new Pose2d(-12.0, -26.0, Math.toRadians(40.0)),Math.toRadians(45.0))
                        .addDisplacementMarker(() -> {
                            if (!robot.shooter.isShooterOn) {
                                autonShooting.shooterOn();
                            }
                        })
                    .splineToSplineHeading(new Pose2d(-10.0, -39.0, robot.shooter.angleToGoal(-10, -39, robot.shooter.redGoal)),Math.toRadians(90.0))


                        .build();

                autonPath.trajectory4 = robot.drive.trajectoryBuilder(autonPath.misswobble.end())
                        //.splineToLinearHeading(new Pose2d(30, isRed * -56, Math.toRadians(260)), 0)
                        .lineToLinearHeading(new Pose2d(10, isRed * -40, Math.toRadians(180)))
                        /*.addDisplacementMarker(20, () -> {
                            wobblePos = 630;
                        })
                        .addDisplacementMarker(new Vector2d(-8,-36).distTo(new Vector2d(15,-38))*.8, () -> {
                            wobblePos=900;
                        })*/
                        .build();
                autonPath.trajectory5 = robot.drive.trajectoryBuilder(autonPath.trajectory4.end())
                        //.splineToLinearHeading(new Pose2d(12, isRed * -52, Math.toRadians(90)), 0)
                        .lineToLinearHeading(new Pose2d(6, isRed * -37, Math.toRadians(180)))
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(0);
                        })
                        .build();
                break;
            case 2: //C
                //TODO: Verify Wobble Goal Position and Ring Height Map
                MinVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(MAX_ANG_VEL),
                        new MecanumVelocityConstraint(15, TRACK_WIDTH)
                ));
                ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
                Vector2d ringVec =new Vector2d (-12,-40);
                Vector2d nowVec=new Vector2d(CurrentP.getX(),CurrentP.getY());
                Vector2d difference=ringVec.minus(nowVec);
                double goAngle = difference.angle();

                autonPath.trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        //.lineToLinearHeading(new Pose2d(-26.0,-31.0,Math.toRadians(300.0)))
                        //.lineToLinearHeading(new Pose2d(-26.0,-33.0,Math.toRadians(320.0)))
                       // .lineToLinearHeading(new Pose2d(ringVec,goAngle),velConstraint,accelConstraint)//goAngle was 325
                        .lineToSplineHeading(new Pose2d(52.0,-46.0,Math.toRadians(130.0)))
                        //  .splineToSplineHeading(new Pose2d(-12.0,-52.0,Math.toRadians(320)),0, new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(10,DriveConstants.TRACK_WIDTH))),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        /*.lineToSplineHeading(new Pose2d(-28.0,-20.0,Math.toRadians(307)))
                        .splineToSplineHeading(new Pose2d(-12.0,-52.0,Math.toRadians(307)),0, new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(10,DriveConstants.TRACK_WIDTH))),new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .addDisplacementMarker(() -> {
                            wobblePos = 650;
                        })
                        .splineToSplineHeading(new Pose2d(48.0,-45.0,Math.toRadians(110.0)),Math.toRadians(320.0))*/
                        .build();
                autonPath.ring2 = robot.drive.trajectoryBuilder(autonPath.trajectory2.end())
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

                autonPath.ring3 = robot.drive.trajectoryBuilder(autonPath.ring2.end())

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
                autonPath.drop1 = robot.drive.trajectoryBuilder(autonPath.ring2.end())
                      //  .splineToSplineHeading(new Pose2d(-20,-43.0,Math.toRadians(270.0)),0)
                        //.splineToSplineHeading(new Pose2d(48.0,-45.0,Math.toRadians(110.0)),Math.toRadians(320.0))
                      //  .lineToSplineHeading(new Pose2d(53.0,-45.0,Math.toRadians(110.0)))

                       // .lineToSplineHeading(new Pose2d(53.0,-45.0,Math.toRadians(110.0)),velConstraint2,accelConstraint2)
                        .lineToSplineHeading(new Pose2d(56.0,-45.0,Math.toRadians(110.0)),velConstraint2,accelConstraint2)
                        /*.addDisplacementMarker(2,() -> {
                            wobblePos = 650;
                        })*/

                        .build();
/*
                trajectory2 = robot.drive.trajectoryBuilder(CurrentP)
                        .lineToLinearHeading(new Pose2d(48, isRed * -45, Math.toRadians(110)))
                        .build();
  */
                autonPath.trajectory3 = robot.drive.trajectoryBuilder(autonPath.trajectory2.end())
                        //.splineTo(new Vector2d(-55, isRed * -55), 0) -20, 44
                  /*      .splineToSplineHeading(new Pose2d(-8, -47 * isRed, Math.toRadians(0)),Math.toRadians(180))
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(700);
                        })
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(900);
                        })
                        .splineToLinearHeading(new Pose2d(-34, -45 * isRed,Math.toRadians(0)),Math.toRadians(180))
                        .build();

                   */
                        .splineToSplineHeading(new Pose2d(-8, -49 * isRed, Math.toRadians(0)),Math.toRadians(180))
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(700);
                        })
                        .addDisplacementMarker(() -> {
                            autonWobble.setWobblePos(900);
                        })
                        .splineToLinearHeading(new Pose2d(-34, -46 * isRed,Math.toRadians(0)),Math.toRadians(180))//was x=-34,-47
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
                /*autonPath.misswobble = robot.drive.trajectoryBuilder(autonPath.trajectory3.end())
                        .splineToLinearHeading(new Pose2d(-8.0, -39.0, Math.toRadians(0.0)),Math.toRadians(90.0))
                        //.splineToSplineHeading(new Pose2d(-12.0, -30.0, Math.toRadians(40.0)),Math.toRadians(45.0))
                      //  .addDisplacementMarker(() -> {
                    //        if (!robot.shooter.isShooterOn) {
                    //            targetVelocity = robot.shooter.shooterOn();
                   //         }
                    //    })
                   // .splineToSplineHeading(new Pose2d(-9.0, -22.0, robot.shooter.angleToGoal(-9, -22, robot.shooter.redGoal)),Math.toRadians(90.0))


                        .build();
*/
                autonPath.trajectory4 = robot.drive.trajectoryBuilder(autonPath.trajectory3.end())
                        .lineToLinearHeading(new Pose2d(36, isRed * -52, Math.toRadians(165)))
                        .addTemporalMarker(2.5, () -> {
                            autonWobble.setWobblePos(550);
                        })
                        .build();
                autonPath.trajectory5 = robot.drive.trajectoryBuilder(autonPath.trajectory4.end())
                        .lineToLinearHeading(new Pose2d(12, isRed * -46,Math.toRadians(90)))
                        .addTemporalMarker(.5, () -> {
                            autonWobble.setWobblePos(0);
                        })

                        .build();
                break;
        }



    }


}

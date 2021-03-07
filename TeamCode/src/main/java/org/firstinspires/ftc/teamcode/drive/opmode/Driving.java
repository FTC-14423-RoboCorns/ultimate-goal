package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAlignWithPoint;
import org.firstinspires.ftc.teamcode.drive.Shooter;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/**
 * This opmode demonstrates how one would implement "align to point behavior" in teleop. You specify
 * a desired vector (x/y coordinate) via`targetPosition`. In the `ALIGN_TO_POINT` mode, the bot will
 * switch into field centric control and independently control its heading to align itself with the
 * specified `targetPosition`.
 * <p>
 * Press `a` to switch into alignment mode and `b` to switch back into standard teleop driving mode.
 * <p>
 * Note: We don't call drive.update() here because it has its own field drawing functions. We don't
 * want that to interfere with our graph so we just directly update localizer instead
 */
@Config
@TeleOp(group = "advanced")
public class Driving extends LinearOpMode {

    private Robot robot;
    public static double DRAWING_TARGET_RADIUS = 2;
    private ElapsedTime buttonWait;
    private ElapsedTime wobbleWait;
    private boolean debug = false;
    private boolean dashboard =true;
    int oneShoot = 0;
    public int wobblePos;
    private boolean manualPowershot=false;
    private boolean twofer=false;

    //Gamepad1
    private boolean driveButtonDown;
    private boolean spitButtonDown;
    private boolean stopShootButtonDown;
    private boolean liftModeButtonDown;
    private boolean intakeButtonDown;
    private boolean wobbleButtonDown;
    private boolean endGameButtonDown;
    private boolean resetOdomButtonDown;
    private boolean resetWobbleButtonDown;
    private boolean onePowershotButtonDown;
    private boolean turnButtonDown;
    //private boolean switchGoalButtonDown;

    //Gamepad2
    private boolean powerShotButtonDown;
    private boolean spinupButtonDown;
    private boolean manualWobbleDecreaseButtonDown;
    private boolean manualWobbleIncreaseButtonDown;
    private boolean shootButtonDown;
    private boolean ringIncreaseButtonDown;
    private boolean ringDecreaseButtonDown;
    private boolean psIncreaseButtonDown;
    private boolean psDecreaseButtonDown;
    private boolean headingRightButtonDown;
    private boolean headingLeftButtonDown;

    private boolean upOrDown = true;

    private int shootNumber = 3;
    private int powershootNumber=3;
    private ElapsedTime waitTimer = new ElapsedTime();
    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT,
        RESET_ODOMETRY,
        POWERSHOT,
        SHOOT
    }

    enum Intake_State {
        INTAKE_OFF,
        INTAKE_ON,
        INTAKE_SPIT
    }

    enum Wobble_State {
        WOBBLE_UP,
        WOBBLE_DOWN,
        WOBBLE_DOWNWAIT,
        WOBBLE_CLOSE,
        WOBBLE_OPEN,
        WOBBLE_UPWAIT
    }

    enum Shooter_State {
        SHOOTER_ON,
        RAMP_UP,
        SHOOT,
        PUSHER_OUT,
        SHOOTER_OFF,
        RESET_SHOOT_COUNT,
        SHOOTER_RESET
    }

    enum PowershotState {
        START,
        TRAJECTORY_1,
        FIRST_TURN,
        SHOOTER_ON,
        SHOOT,
        TURN,
        TURN2,
        IDLE
    }
    PowershotState endGame = PowershotState.IDLE;
    PowershotState manualGame = PowershotState.IDLE;


    double targetVelocity = 2000;
    private Intake_State intakeMode = Intake_State.INTAKE_OFF;
    private Mode currentMode = Mode.NORMAL_CONTROL;
    private Shooter_State shooterMode = Shooter_State.SHOOTER_OFF;
    private Wobble_State wobbleMode = Wobble_State.WOBBLE_DOWN;
    private int shootCount = 0;
   private  ElapsedTime waitTimer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int isRed = 1;
    Trajectory powerTraj,shootTraj,strafe1,strafe2;
/*
    private static final double POWEROFFSET = Math.toRadians(6.5);
    private static final double POWEROFFSET2 = Math.toRadians(6);
    private static final double POWEROFFSET3 = Math.toRadians(5);
*/
    private static final double POWEROFFSET = 0;
    private static final double POWEROFFSET2 = 0;
    private static final double POWEROFFSET3 = 0;
    private TelemetryPacket packet;
    private Canvas fieldOverlay;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        robot = new Robot(hardwareMap, telemetry,true);
        robot.drive.speedMult=1;//use this to modulate speed
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class, so must be first called after initializing class
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter.liftState= Shooter.LiftState.DYNAMIC;
       // robot.shooter.SHOOTER_OFFSET=5; may  not need since adjusting goal target
        checkButtons();
        /*handled in checkButtons
        driveButtonDown = false;
        intakeButtonDown = false;
        shootButtonDown = false;
        spinupButtonDown=false;

         */
        //Used to wait between button inputs
        buttonWait = new ElapsedTime();
        wobbleWait = new ElapsedTime();

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //TODO: Need way to define whether read or blue? Need manual if PoseStorage is corrupt
        //Pose2d startPose = new Pose2d(-63, -24 * isRed, 0);
        //robot.drive.getLocalizer().setPoseEstimate(startPose);

       robot.drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        System.out.println("start pose "+PoseStorage.currentPose);
        //isRed = PoseStorage.isRed;

        shooterMode = Shooter_State.SHOOTER_OFF;
        wobbleMode = Wobble_State.WOBBLE_DOWN;

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
        //TODO:need to adjust for red or blue
//        Vector2d targetPosition = new Vector2d(robot.shooter.redGoal.x, ((isRed*robot.shooter.redGoal.y) +robot.shooter.SHOOTER_OFFSET)); //offset to adjust for shooter position on robot
  //new tangent code
        Vector2d targetPosition = new Vector2d(robot.shooter.redGoal.x, ((isRed*robot.shooter.redGoal.y))); //offset to adjust for shooter position on robot
        if (isRed==1)  robot.shooter.currentTarget = robot.shooter.redGoal;
                else robot.shooter.currentTarget = robot.shooter.blueGoal;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            //need this at beginning of each loop for bulk reads. Manual mode set in robot class
            for (LynxModule module : robot.allHubs) {
                module.clearBulkCache();
            }
            // Read pose
            Pose2d poseEstimate = robot.drive.getLocalizer().getPoseEstimate();

            //update shooter
            robot.shooter.update(poseEstimate);

            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();

            telemetry.addData("mode", currentMode);

            if (dashboard) {
                // Declare telemetry packet for dashboard field drawing
                packet = new TelemetryPacket();
                fieldOverlay = packet.fieldOverlay();
            }
            checkButtons();

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if 'rb` is pressed
                    if (gamepad1.right_bumper && !driveButtonDown) {
                        driveButtonDown = true;
                        if (!robot.shooter.isShooterOn) {
                            targetVelocity = robot.shooter.shooterOn();
                        }
                        turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redGoal));
                        currentMode = Mode.ALIGN_TO_POINT;

                    }

                    if (gamepad1.left_bumper && !onePowershotButtonDown) {
                        onePowershotButtonDown = true;
                        manualPowershot = true;
                        twofer=false;
                        shootCount = 0;
                        robot.shooter.currentTarget = robot.shooter.redPowerShot1;
                        powershootNumber = 1;
                        if (!robot.shooter.isShooterOn) {
                            targetVelocity = robot.shooter.shooterOn(1850);
                            if (debug) System.out.println("SHOOTER ON target vel " + targetVelocity);
                        }
                        endGame = PowershotState.TRAJECTORY_1;
                        currentMode = Mode.POWERSHOT;
                    }

                    if (gamepad1.a&&!turnButtonDown) {
                         if (gamepad1.right_trigger > .5) {
                            turnButtonDown = true;
                            manualPowershot = false;
                            twofer = true;
                            shootCount = 0;
                            robot.shooter.currentTarget = robot.shooter.redPowerShot1;
                            powershootNumber = 2;
                            if (!robot.shooter.isShooterOn) {
                                targetVelocity = robot.shooter.shooterOn(1850);
                                if (debug)
                                    System.out.println("SHOOTER ON target vel " + targetVelocity);
                            }
                            endGame = PowershotState.TRAJECTORY_1;
                            currentMode = Mode.POWERSHOT;
                        }
                    }


                    if (gamepad2.left_bumper && !spinupButtonDown) {
                        spinupButtonDown = true;
                        /*shootTraj = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                                //    .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), 0)
                                .lineToLinearHeading(new Pose2d(-10, isRed * -8, robot.shooter.angleToGoal(-10, -8, robot.shooter.redGoal)-POWEROFFSET))

                                .build();*/
                        if (!robot.shooter.isShooterOn) {
                            targetVelocity = robot.shooter.shooterOn();
                        }
                        /*
                        robot.drive.followTrajectoryAsync(shootTraj);*/

                    }


                    if (gamepad2.b&&!powerShotButtonDown) {
                        if (gamepad2.right_trigger > .5) {
                            powershootNumber = 3;
                            manualPowershot = false;
                            robot.shooter.currentTarget = robot.shooter.redPowerShot1; //affects lift height - change if we shoot for goal
                            if (!robot.shooter.isShooterOn) {
                                targetVelocity = robot.shooter.shooterOn();
                            }
                            powerTraj = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                                    //    .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), 0)
                                    .lineToLinearHeading(new Pose2d(-7, isRed * -24.5, 0))

                                    .build();
                            endGame = PowershotState.START;
                            currentMode = Mode.POWERSHOT;
                        }
                    }


                    if (gamepad2.right_stick_button && !resetOdomButtonDown) {
                        resetOdomButtonDown = true;
                        currentMode = Mode.RESET_ODOMETRY;
                    }

                    // Standard teleop control
                    // Convert gamepad input into desired pose velocity
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );
                 //   robot.drive.speedMult= Range.clip(.8+(.4-gamepad1.right_trigger),1,.4);//two numbers should add to 1, subtraction is the min drive multiplier
                    robot.drive.setWeightedDrivePower(driveDirection);
                    break;

                case POWERSHOT:
                  //  System.out.println("SHOOTER in POWERSHOT state");
                    if (gamepad1.right_bumper && !driveButtonDown) {
                    //    System.out.println("SHOOTER Powershot canceled");
                        //robot.shooter.currentTarget=robot.shooter.redGoal;
                        driveButtonDown = true;
                        robot.drive.cancelFollowing();
                        robot.shooter.shooterOff();
                        robot.shooter.pusherOut();
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    break;

                case SHOOT:
                    //  System.out.println("SHOOTER in POWERSHOT state");
                    if (gamepad1.right_bumper && !driveButtonDown) {
                        //    System.out.println("SHOOTER Powershot canceled");
                        //robot.shooter.currentTarget=robot.shooter.redGoal;
                        driveButtonDown = true;
                        robot.drive.cancelFollowing();
                        robot.shooter.shooterOff();
                        robot.shooter.pusherOut();
                        currentMode = Mode.NORMAL_CONTROL;
                    }
                    if (!robot.drive.isBusy())
                    {
                        if (debug) {
                            System.out.println("SHOOTER_TURNto Actual  " + Math.toDegrees(robot.drive.getLocalizer().getPoseEstimate().getHeading()));
                        }
                        //TODO Decide whether to autoshoot
                        // shooterMode = Shooter_State.SHOOTER_ON;
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    break;

                case ALIGN_TO_POINT:
                    // Switch back into normal driver control mode if `rb` is pressed
                    if (gamepad1.right_bumper && !driveButtonDown) {
                        driveButtonDown = true;
                        robot.drive.cancelFollowing();
                        currentMode = Mode.NORMAL_CONTROL;
                    }


                    if (!robot.drive.isBusy())
                        {
                            if (debug) {
                                System.out.println("SHOOTER_TURNto Actual  " + Math.toDegrees(robot.drive.getLocalizer().getPoseEstimate().getHeading()));
                            }
                            currentMode = Mode.NORMAL_CONTROL;
                        }

                    /* Commenting align to point out to do turn
                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    //Vector2d offset = new Vector2d(0, 4.75);
                   // Vector2d adjusted = poseEstimate.vec().plus(offset);
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    //Vector2d difference = targetPosition.minus(adjusted);

                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );
*/
                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());




                    break;
                case RESET_ODOMETRY:
                    //TODO check Y value for powershot
                    Pose2d newPose = new Pose2d(36, -61 * isRed, 0); //empty square
                   // Pose2d newPose = new Pose2d(9, -61 * isRed, 0);//old red square
                    robot.drive.getLocalizer().setPoseEstimate(newPose);
                    currentMode = Mode.NORMAL_CONTROL;
                    break;
            }
            //handleShootMode();
            handleIntake();
            handleWobble();

            handlePowershot();
            handleChangeHeading();
            if (gamepad2.y && !stopShootButtonDown)
            {
                stopShootButtonDown=true;
                shooterMode = Shooter_State.SHOOTER_RESET;

                if (currentMode==Mode.POWERSHOT) {
                    robot.drive.cancelFollowing();
                    robot.shooter.shooterOff();
                    robot.shooter.pusherOut();
                    endGame= PowershotState.IDLE;
                    currentMode = Mode.NORMAL_CONTROL;
                }
            }
            switch (shooterMode) {

                case SHOOTER_OFF:


                    if (gamepad2.x && !shootButtonDown)
                    // if (oneShoot == 0)
                    {
                        robot.shooter.currentTarget=robot.shooter.redGoal;
                        oneShoot = 1;
                        shootButtonDown = true;
                        if (debug) System.out.println("SHOOT_X");
                        shooterMode = Shooter_State.SHOOTER_ON;
                    }
                    break;

                case SHOOTER_ON:
                    if (debug) System.out.println("SHOOT_Shooter On");
                    if (!robot.shooter.isShooterOn) {
                        targetVelocity = robot.shooter.shooterOn();
                    }
                    shooterMode = Shooter_State.RAMP_UP;
                    break;

                case RAMP_UP:
                    if (debug)
                        System.out.println("SHOOT_Ramp In " + robot.shooter.shooter.getVelocity());
                    if (!robot.shooter.isShooterReady(targetVelocity - 200)||waitTimer.milliseconds()>500) { //was 500 test faster
                        if (debug)
                            System.out.println("SHOOT_Check Shoot " + targetVelocity + " " + robot.shooter.shooter.getVelocity());
                        robot.shooter.pusherOut();
                        waitTimer.reset();
                    }
                    if  (!robot.drive.isBusy()) {
                        if (debug) System.out.println("SHOOT_Shoot Count " + shootCount);
                        if (debug) System.out.println("SHOOT_Shoot Number " + shootNumber);
                        if (shootCount <= shootNumber) {

                            if (robot.shooter.isShooterReady(targetVelocity)) {
                                if (debug)
                                    System.out.println("SHOOT_Shooter Ready " + robot.shooter.shooter.getVelocity());
                                if (debug)
                                    System.out.println("SHOOT_Shooter Ready " + robot.shooter.isShooterReady(targetVelocity));
                                if (!robot.shooter.shooting && waitTimer.milliseconds() > 400) { //was 400 testing faster
                                    shootCount += 1;
                                    shooterMode = Shooter_State.SHOOT;
                                    if (debug) System.out.println("SHOOT_Shooting");
                                }
                            }
                        } else {

                            shooterMode = Shooter_State.SHOOTER_RESET;
                            shootNumber = 3;
                            currentMode = Mode.NORMAL_CONTROL;
                            robot.shooter.pusherOut();
                            if (debug) System.out.println("SHOOT_Done");
                        }
                    }
                    break;

                case SHOOT:
                    robot.shooter.pusherIn();
                    waitTimer.reset();
                    shooterMode = Shooter_State.RAMP_UP;
                    if (debug) System.out.println("SHOOT_Done");
                    break;

                case SHOOTER_RESET:
                    robot.shooter.shooterOff();
                    robot.shooter.pusherOut();
                    shootCount = 0;
                    shooterMode=Shooter_State.SHOOTER_OFF;
                    break;
            }
            if (gamepad2.dpad_right && !ringIncreaseButtonDown) {
                ringIncreaseButtonDown = true;
                shootNumber += 1;

            }
            if (gamepad2.dpad_left && !ringDecreaseButtonDown) {
                ringDecreaseButtonDown = true;
                shootNumber += 1;
            }
    if (dashboard) {
        // Draw bot on canvas
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
    }


            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());

            // Update he localizer
            robot.drive.getLocalizer().update();
            robot.drive.update();
        if(dashboard) {
            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
            /*telemetry.addData("angle", robot.shooter.angle);
            telemetry.addData("shooter Height", robot.shooter.shooterHeight);
            telemetry.addData("crank Angle", robot.shooter.crankAngle);*/
            telemetry.addData("lift Pos", robot.shooter.liftPos);
            //telemetry.addData("shooterReady", robot.shooter.isShooterReady(targetVelocity));
            //telemetry.addData("targetvelocity", targetVelocity);
            //telemetry.addData("shoot_count", shootCount);
            //telemetry.addData("lift Pos", shooterMode);

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("shooterNumber", shootNumber);

            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }

    public void handleChangeHeading() {
        Pose2d adj=new Pose2d(0,0,Math.toRadians(2));
        if(gamepad1.dpad_right &&!headingRightButtonDown)

    {
        headingRightButtonDown=true;
        Pose2d newPose = robot.drive.getLocalizer().getPoseEstimate().minus(adj);
        robot.drive.getLocalizer().setPoseEstimate(newPose);
        turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redGoal));
        currentMode = Mode.ALIGN_TO_POINT;
    }
        if(gamepad1.dpad_left &&!headingLeftButtonDown)

    {
        headingLeftButtonDown=true;
        Pose2d newPose = robot.drive.getLocalizer().getPoseEstimate().plus(adj);
        robot.drive.getLocalizer().setPoseEstimate(newPose);
        turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redGoal));
        currentMode = Mode.ALIGN_TO_POINT;
    }

}

        public void checkButtons(){
        if (!gamepad1.right_bumper) driveButtonDown=false;
            if (!gamepad1.left_bumper) onePowershotButtonDown=false;
        if (!gamepad2.a) intakeButtonDown=false;
            if (!gamepad1.a) turnButtonDown=false;
        if (!gamepad1.y) spitButtonDown=false;
        if (!gamepad1.b) wobbleButtonDown=false;
        if (!gamepad1.x) endGameButtonDown=false;
        if (!gamepad1.dpad_right) headingRightButtonDown=false;
        if (!gamepad1.dpad_left) headingLeftButtonDown=false;

        if (!gamepad2.right_stick_button) resetOdomButtonDown=false;
        if (!gamepad2.left_stick_button) resetWobbleButtonDown=false;
        if (!gamepad2.x) shootButtonDown=false;
        if (!gamepad2.b) powerShotButtonDown=false;
        if (!gamepad2.y) stopShootButtonDown=false;
        if (!gamepad2.a) liftModeButtonDown=false;
        if (!gamepad2.dpad_down) manualWobbleDecreaseButtonDown=false;
        if (!gamepad2.dpad_up) manualWobbleIncreaseButtonDown=false;
        if (!gamepad2.left_bumper) spinupButtonDown=false;
        //if (!gamepad2.right_bumper) ringIncreaseButtonDown=false;
        //if (!gamepad2.left_bumper) ringDecreaseButtonDown=false;
        if (!gamepad2.dpad_right) psIncreaseButtonDown=false;
        if (!gamepad2.dpad_left) psDecreaseButtonDown=false;

    }

    /*public void handleShootMode() {
        if (gamepad2.a && !liftModeButtonDown){
            liftModeButtonDown=true;
            if (robot.shooter.liftState== Shooter.LiftState.STATIC) {
                robot.shooter.liftState= Shooter.LiftState.DYNAMIC;
            } else {
                robot.shooter.liftState= Shooter.LiftState.STATIC;
            }
        }
    }*/



    public void handleIntake() {
        switch (intakeMode) {
            case INTAKE_OFF:
                if (gamepad2.a && !intakeButtonDown) {
                    intakeButtonDown=true;
                    robot.intake.turnOn();
                    intakeMode = Intake_State.INTAKE_ON;
                }
                if (gamepad1.y && !spitButtonDown) {
                    spitButtonDown=true;
                    robot.intake.spit();
                    intakeMode = Intake_State.INTAKE_SPIT;
                }
                break;
            case INTAKE_ON:
                if (gamepad2.a && !intakeButtonDown) {
                    intakeButtonDown=true;
                    robot.intake.turnOff();
                    intakeMode = Intake_State.INTAKE_OFF;
                }
                if (gamepad1.y && !spitButtonDown) {
                    spitButtonDown=true;
                    intakeMode = Intake_State.INTAKE_SPIT;
                    robot.intake.spit();
                }
                break;
            case INTAKE_SPIT:
                if (gamepad2.a && !intakeButtonDown) {
                    intakeButtonDown=true;
                    robot.intake.turnOn();
                    intakeMode = Intake_State.INTAKE_ON;
                }
                if (gamepad1.y && !spitButtonDown) {
                    spitButtonDown=true;
                    intakeMode = Intake_State.INTAKE_ON;
                    robot.intake.turnOn();
                }
                break;
        }
    }

    public void handleWobble() {

        if (gamepad2.dpad_down&& !manualWobbleDecreaseButtonDown){
            manualWobbleDecreaseButtonDown=true;
            if (gamepad2.left_trigger>.5) robot.wobble.override=true;
            else robot.wobble.override=false;
            wobblePos+=25;
            robot.wobble.wobbleMovetoPosition(wobblePos);
        }

        if (gamepad2.dpad_up&& !manualWobbleIncreaseButtonDown){
            manualWobbleIncreaseButtonDown=true;
            if (gamepad2.left_trigger>.5) robot.wobble.override=true;
                    else robot.wobble.override=false;
            wobblePos-=25;
            robot.wobble.wobbleMovetoPosition(wobblePos);
        }

        if (gamepad2.left_stick_button && !resetWobbleButtonDown){
            resetWobbleButtonDown=true;
            robot.wobble.resetWobble();
        }


        switch (wobbleMode){
            case WOBBLE_DOWN:
                //System.out.println("** Wobble got to case WOBBLE_DOWN");
                if (gamepad1.b && !wobbleButtonDown){
                   // robot.wobble.lowerWobbleFromFront();
                    //System.out.println("Wobble position " + robot.wobble.wobble.getCurrentPosition());
                    wobbleButtonDown = true;
                    wobblePos=875;
                    robot.wobble.wobbleMovetoPosition(wobblePos);
                  //  robot.wobble.wobbleSetRaise(wobblePos);//should no longer be necessary
                    //robot.wobble.fastMovetoPos(900);
                    wobbleMode=Wobble_State.WOBBLE_DOWNWAIT;
                }
                if (gamepad1.x && !endGameButtonDown){
                    endGameButtonDown = true;
                    robot.wobble.openClaw();
                    wobbleMode=Wobble_State.WOBBLE_UP;
                    wobbleWait.reset();
                }

                break;
            case WOBBLE_DOWNWAIT:
                //wobblePos=900;

                if (gamepad1.b && !wobbleButtonDown){
                    // robot.wobble.lowerWobbleFromFront();
                    //System.out.println("Wobble position " + robot.wobble.wobble.getCurrentPosition());
                    wobbleButtonDown = true;

                    //  robot.wobble.wobbleSetRaise(wobblePos);//should no longer be necessary
                    //robot.wobble.fastMovetoPos(900);
                    wobbleMode=Wobble_State.WOBBLE_DOWN;
                }
                if (gamepad1.x && !endGameButtonDown){
                    endGameButtonDown = true;
                    robot.wobble.openClaw();
                    wobbleMode=Wobble_State.WOBBLE_UP;
                    wobbleWait.reset();
                }

                if (robot.wobble.isWobbleThere(wobblePos))
                {
                        wobbleMode = Wobble_State.WOBBLE_OPEN;
                }
                break;
            case WOBBLE_OPEN:
                robot.wobble.openClaw();
                wobbleMode=Wobble_State.WOBBLE_CLOSE;
                break;
            case WOBBLE_CLOSE:
                if (gamepad1.b && !wobbleButtonDown){
                    wobbleButtonDown = true;
                    robot.wobble.closeClaw();
                    wobbleMode=Wobble_State.WOBBLE_UP;
                    wobbleWait.reset();
                }
                if (gamepad1.x && !endGameButtonDown){
                    endGameButtonDown = true;
                    robot.wobble.openClaw();
                    wobbleMode=Wobble_State.WOBBLE_UP;
                    wobbleWait.reset();
                }

                break;
            case WOBBLE_UP:
                if (wobbleWait.milliseconds() > 500)
                {
                    //robot.wobble.raiseWobble();
                    wobblePos=500;
                    //robot.wobble.wobbleSetRaise(wobblePos);//should not need
                    robot.wobble.wobbleMovetoPosition(wobblePos);
                    wobbleMode = Wobble_State.WOBBLE_UPWAIT;
                }
                break;

            case WOBBLE_UPWAIT:
                //wobblePos=500
                if (gamepad1.b && !wobbleButtonDown){
                    // robot.wobble.lowerWobbleFromFront();
                    //System.out.println("Wobble position " + robot.wobble.wobble.getCurrentPosition());
                    wobbleButtonDown = true;

                    //  robot.wobble.wobbleSetRaise(wobblePos);//should no longer be necessary
                    //robot.wobble.fastMovetoPos(900);
                    wobbleMode=Wobble_State.WOBBLE_UP;
                }
                if (gamepad1.x && !endGameButtonDown){
                    endGameButtonDown = true;
                    robot.wobble.openClaw();
                    wobbleMode=Wobble_State.WOBBLE_UP;
                    wobbleWait.reset();
                }




                if (robot.wobble.isWobbleThere(wobblePos)) {
                    wobbleMode = Wobble_State.WOBBLE_DOWN;
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

        if (debug)  System.out.println("SHOOTER_targetAngle (in Degrees) " + Math.toDegrees(normAngle));

        // double diff= normAngle-currentHeading;
      /* if (debug) {
      System.out.println("SHOOTER_Turnto Current  " + Math.toDegrees(currentHeading));
        System.out.println("SHOOTER_Turnto Target  " + Math.toDegrees(targetAngle));
        System.out.println("SHOOTER_Turnto Turn  " + Math.toDegrees(normAngle));
        }*/
        robot.drive.turnAsync(normAngle);
        //robot.drive.turnAsync(targetAngle);
    }
    public void turnTo(double targetAngle)
    {
        turnTo(targetAngle, true);
    }

    void handlePowershot() {


        switch (endGame) {
            case IDLE:

                break;

            case START:
                robot.drive.followTrajectoryAsync(powerTraj);
                if (!robot.shooter.isShooterOn) {
                    targetVelocity = robot.shooter.shooterOn();
                    if (debug) System.out.println("SHOOTER ON target vel " + targetVelocity);
                }
                endGame = PowershotState.TRAJECTORY_1;
                break;

            //TODO: CHANGE ORDER FOR OUTSIDE RED
            case TRAJECTORY_1:

                if (!robot.drive.isBusy()) {
                    if (debug)
                        System.out.println("SHOOTER_FIRSTTURN_X " + robot.drive.getPoseEstimate().getX());
                    if (debug)
                        System.out.println("SHOOTER_FIRSTTURN_Y " + robot.drive.getPoseEstimate().getY());
                    // turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1)-POWEROFFSET);
                    if (twofer)  robot.drive.turnAsync(Math.toRadians(4.5));
                        else if (!manualPowershot) turnTo(0);
                    endGame = PowershotState.FIRST_TURN;
                    if (debug)
                        System.out.println("SHOOTER_firstAngle " + Math.toDegrees(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1)));
                }
                break;

            case FIRST_TURN:
                // System.out.println("SHOOTER Waiting for turn");
                if (!robot.drive.isBusy()) {
                    if (debug) System.out.println("SHOOTER Turn done moving to on");
                    endGame = PowershotState.SHOOTER_ON;
                }
                break;

            case SHOOTER_ON:
                // System.out.println("SHOOTER_shooterOn");
                //System.out.println("SHOOTER Waiting for target vel");
                waitTimer1.reset();
                if (robot.shooter.isShooterReady(targetVelocity)) {
                    if (debug) System.out.println("SHOOTER target vel " + targetVelocity);
                    endGame = PowershotState.SHOOT;
                } //will need to add a timer later to move on in case we never get up to speed
                break;

            case SHOOT:
                boolean done;

                // if (debug) System.out.println("SHOOTER_shootInState");
                // if (debug) System.out.println("SHOOTER_shootInState still turning " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                if (!robot.drive.isBusy() && waitTimer1.milliseconds() > 400) {//making sure our turn is done  && waitTimer1.time()>1500
                   /* if (isRed==1) {
                        done=robot.drive.getPoseEstimate().getHeading() <= Math.toRadians(PowerTarget);
                    } else {
                        done=robot.drive.getPoseEstimate().getHeading()>= Math.toRadians(PowerTarget);
                    }
                    if (done) {*/
                    //if (debug) System.out.println("SHOOTER_now shooting heading " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                    if (shootCount < powershootNumber) {
                        //if (debug)
                            System.out.println("SHOOTER_shoot " + shootCount);
                            System.out.println("SHOOTER_powershoot_NUMBER " + powershootNumber);

                        if (debug)
                            System.out.println("SHOOTER_Final heading" + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                        //   System.out.println("SHOOT_Shooter Ready " + robot.shooter.shooter.getVelocity());
                        //   System.out.println("SHOOT_Shooter Ready " + robot.shooter.isShooterReady(targetVelocity));
                        robot.shooter.pusherIn();
                        shootCount += 1;
                        waitTimer1.reset();
                    }
                    if (shootCount < powershootNumber) {
                        //System.out.println("SHOOTER_shootToTurn");
                        endGame = PowershotState.TURN;
                    } else {
                        if (!robot.shooter.isShooterReady(targetVelocity - 200) || waitTimer1.time() >= 300) {//was 500 should be shorter since only waiting for reshoot
                            currentMode = Mode.NORMAL_CONTROL;

                            if (manualPowershot)
                                robot.shooter.pusherOut(); //leave on since we probably want to shoot again
                            else
                                shooterMode = Shooter_State.SHOOTER_RESET; //turns everything off and sends pusher out
                            endGame = PowershotState.IDLE;
                        }
                    }
                }
                break;

            case TURN:
                // System.out.println("SHOOTER_turnInState");
                if (!robot.shooter.isShooterReady(targetVelocity - 200) || waitTimer1.time() >= 400) {
                    if (twofer) {
                            endGame=PowershotState.TRAJECTORY_1;
                            robot.shooter.pusherOut();
                    } else {
                        if (debug) System.out.println("SHOOTER_ringShot");
                        robot.shooter.pusherOut();
                        powerTurn();
                        endGame = PowershotState.TURN2;
                    }
                    //SHOOTER_ON;
                }
                break;
            case TURN2:
                if (!robot.drive.isBusy()) {
                    turnTo(0);
                    endGame = PowershotState.SHOOTER_ON;
                }
                break;


        }

    }

    public void powerTurn()
    {
        // PowerTarget=PowerTarget - (isRed*6);
        //turnTo(isRed*-6);

        if(isRed == 1)
        {
            if(shootCount == 1)
            {
               /*strafe1 = robot.drive.trajectoryBuilder(powerTraj.end())
                        .strafeLeft(8)
                        .build();
                robot.drive.followTrajectoryAsync(strafe1);

                */
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot2)-POWEROFFSET2);
            }
            else
            {
                 /*strafe2 = robot.drive.trajectoryBuilder(strafe1.end())
                        .strafeLeft(8.5)
                        .build();
                robot.drive.followTrajectoryAsync(strafe2);

                  */
               turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot3)-POWEROFFSET3);
            }
        }
        else
        {
            if(shootCount == 1)
            {
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.bluePowerShot2)-POWEROFFSET);
            }
            else
            {
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.bluePowerShot3)-POWEROFFSET);
            }
        }
    }

}


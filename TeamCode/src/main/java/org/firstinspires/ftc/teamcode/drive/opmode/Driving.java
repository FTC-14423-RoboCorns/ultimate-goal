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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAlignWithPoint;
import org.firstinspires.ftc.teamcode.drive.Shooter;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
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
    private boolean debug = true;
    int oneShoot = 0;

    //Gamepad1
    private boolean driveButtonDown;
    private boolean spitButtonDown;
    private boolean stopShootButtonDown;
    private boolean liftModeButtonDown;
    private boolean intakeButtonDown;
    private boolean wobbleButtonDown;
    private boolean endGameButtonDown;
    private boolean resetOdomButtonDown;
    //private boolean switchGoalButtonDown;

    //Gamepad2
    private boolean powerShotButtonDown;
    private boolean manualShooterDecreaseButtonDown;
    private boolean manualShooterIncreaseButtonDown;
    private boolean shootButtonDown;
    private boolean ringIncreaseButtonDown;
    private boolean ringDecreaseButtonDown;
    private boolean psIncreaseButtonDown;
    private boolean psDecreaseButtonDown;
    private boolean headingRightButtonDown;
    private boolean headingLeftButtonDown;

    private boolean upOrDown = true;

    private int shootNumber = 3;
    private ElapsedTime waitTimer = new ElapsedTime();
    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT,
        RESET_ODOMETRY,
        POWERSHOT
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

    enum powershotState {
        START,
        TRAJECTORY_1,
        FIRST_TURN,
        SHOOTER_ON,
        SHOOT,
        TURN,
        IDLE
    }
    powershotState endGame = powershotState.IDLE;


    double targetVelocity = 2000;
    private Intake_State intakeMode = Intake_State.INTAKE_OFF;
    private Mode currentMode = Mode.NORMAL_CONTROL;
    private Shooter_State shooterMode = Shooter_State.SHOOTER_OFF;
    private Wobble_State wobbleMode = Wobble_State.WOBBLE_DOWN;
    private int shootCount = 0;
   private  ElapsedTime waitTimer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int isRed = 1;
    Trajectory powerTraj;
    private static final double POWEROFFSET = Math.toRadians(6.5);
    private static final double POWEROFFSET2 = Math.toRadians(6);
    private static final double POWEROFFSET3 = Math.toRadians(5);

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        robot = new Robot(hardwareMap, telemetry);
        //need this at beginning of each loop for bulk reads. Manual mode set in robot class, so must be first called after initializing class
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveButtonDown = false;
        intakeButtonDown = false;
        shootButtonDown = false;
        //Used to wait between button inputs
        buttonWait = new ElapsedTime();
        wobbleWait = new ElapsedTime();

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //TODO: Need way to define whether read or blue? Need manual if PoseStorage is corrupt
        //Pose2d startPose = new Pose2d(-63, -24 * isRed, 0);
        //robot.drive.getLocalizer().setPoseEstimate(startPose);

       robot.drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        //isRed = PoseStorage.isRed;

        shooterMode = Shooter_State.SHOOTER_OFF;
        wobbleMode = Wobble_State.WOBBLE_DOWN;

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
        //TODO:need to adjust for red or blue
        Vector2d targetPosition = new Vector2d(robot.shooter.redGoal.x, ((isRed*robot.shooter.redGoal.y) +robot.shooter.SHOOTER_OFFSET)); //offset to adjust for shooter position on robot
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

            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

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

                    if (gamepad2.b&&!powerShotButtonDown) {

                        robot.shooter.currentTarget=robot.shooter.redPowerShot1; //affects lift height - change if we shoot for goal
                       powerTraj = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), 0)
                                //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
                                .build();
                        endGame=powershotState.START;
                        currentMode=Mode.POWERSHOT;
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
                    robot.drive.setWeightedDrivePower(driveDirection);
                    break;

                case POWERSHOT:
                  //  System.out.println("SHOOTER in POWERSHOT state");
                    if (gamepad1.right_bumper && !driveButtonDown) {
                    //    System.out.println("SHOOTER Powershot canceled");
                        robot.shooter.currentTarget=robot.shooter.redGoal;
                        driveButtonDown = true;
                        robot.drive.cancelFollowing();
                        robot.shooter.shooterOff();
                        robot.shooter.pusherOut();
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
                            System.out.println("SHOOTER_TURNto Actual  " + Math.toDegrees(robot.drive.getLocalizer().getPoseEstimate().getHeading()));
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
                    Pose2d newPose = new Pose2d(9, -61 * isRed, 0);
                    robot.drive.getLocalizer().setPoseEstimate(newPose);
                    currentMode = Mode.NORMAL_CONTROL;
                    break;
            }
            handleShootMode();
            handleIntake();
            handleWobble();

            handlePowershot();
            handleChangeHeading();
            if (gamepad2.y && !stopShootButtonDown)
            {
                stopShootButtonDown=true;
                shooterMode = Shooter_State.SHOOTER_RESET;
            }
            switch (shooterMode) {

                case SHOOTER_OFF:


                    if (gamepad2.x && !shootButtonDown)
                    // if (oneShoot == 0)
                    {
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
                    if (!robot.shooter.isShooterReady(targetVelocity - 200)) {
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
                                if (!robot.shooter.shooting && waitTimer.milliseconds() > 400) {
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

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);



            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());

            // Update he localizer
            robot.drive.getLocalizer().update();
            robot.drive.update();

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
        Pose2d adj=new Pose2d(0,0,2);
        if(gamepad1.dpad_right &&!headingRightButtonDown)

    {

        Pose2d newPose = robot.drive.getLocalizer().getPoseEstimate().plus(adj);
        robot.drive.getLocalizer().setPoseEstimate(newPose);
    }
        if(gamepad1.dpad_left &&!headingLeftButtonDown)

    {
        Pose2d newPose = robot.drive.getLocalizer().getPoseEstimate().minus(adj);
        robot.drive.getLocalizer().setPoseEstimate(newPose);
    }

}

        public void checkButtons(){
        if (!gamepad1.right_bumper) driveButtonDown=false;
        if (!gamepad1.a) intakeButtonDown=false;
        if (!gamepad1.y) spitButtonDown=false;
        if (!gamepad1.b) wobbleButtonDown=false;
        if (!gamepad1.x) endGameButtonDown=false;
        if (!gamepad1.dpad_right) headingRightButtonDown=false;
        if (!gamepad1.dpad_left) headingLeftButtonDown=false;

        if (!gamepad2.right_stick_button) resetOdomButtonDown=false;

        if (!gamepad2.x) shootButtonDown=false;
        if (!gamepad2.b) powerShotButtonDown=false;
        if (!gamepad2.y) stopShootButtonDown=false;
        if (!gamepad2.a) liftModeButtonDown=false;
        if (!gamepad2.dpad_down) manualShooterDecreaseButtonDown=false;
        if (!gamepad2.dpad_up) manualShooterIncreaseButtonDown=false;
        if (!gamepad2.right_bumper) ringIncreaseButtonDown=false;
        if (!gamepad2.left_bumper) ringDecreaseButtonDown=false;
        if (!gamepad2.dpad_right) psIncreaseButtonDown=false;
        if (!gamepad2.dpad_left) psDecreaseButtonDown=false;

    }

    public void handleShootMode() {
        if (gamepad2.a && !liftModeButtonDown){
            liftModeButtonDown=true;
            if (robot.shooter.liftState== Shooter.LiftState.STATIC) {
                robot.shooter.liftState= Shooter.LiftState.DYNAMIC;
            } else {
                robot.shooter.liftState= Shooter.LiftState.STATIC;
            }
        }
    }

    public void handleIntake() {
        switch (intakeMode) {
            case INTAKE_OFF:
                if (gamepad1.a && !intakeButtonDown) {
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
                if (gamepad1.a && !intakeButtonDown) {
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
                if (gamepad1.a && !intakeButtonDown) {
                    intakeButtonDown=true;
                    robot.intake.turnOn();
                    intakeMode = Intake_State.INTAKE_ON;
                }
                if (gamepad1.y && !spitButtonDown) {
                    spitButtonDown=true;
                    intakeMode = Intake_State.INTAKE_OFF;
                    robot.intake.turnOff();
                }
                break;
        }
    }

    public void handleWobble() {
        switch (wobbleMode){
            case WOBBLE_DOWN:
                //System.out.println("** Wobble got to case WOBBLE_DOWN");
                if (gamepad1.b && !wobbleButtonDown){
                   // robot.wobble.lowerWobbleFromFront();
                    //System.out.println("Wobble position " + robot.wobble.wobble.getCurrentPosition());
                    wobbleButtonDown = true;
                    robot.wobble.wobbleSetRaise(900);
                    //robot.wobble.fastMovetoPos(900);
                    wobbleMode=Wobble_State.WOBBLE_DOWNWAIT;
                }
                break;
            case WOBBLE_DOWNWAIT:
                robot.wobble.wobbleMovetoPosition(900);
                if (robot.wobble.isWobbleThere(900))
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
                    wobbleMode=Wobble_State.WOBBLE_UP;
                    wobbleWait.reset();
                }

                break;
            case WOBBLE_UP:
                if (wobbleWait.milliseconds() > 500)
                {
                    //robot.wobble.raiseWobble();
                    robot.wobble.wobbleSetRaise(450);

                    wobbleMode = Wobble_State.WOBBLE_UPWAIT;
                }
                break;

            case WOBBLE_UPWAIT:
                robot.wobble.wobbleMovetoPosition(450);
                if (robot.wobble.isWobbleThere(450)) {
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

        System.out.println("SHOOTER_targetAngle (in Degrees) " + Math.toDegrees(normAngle));

        // double diff= normAngle-currentHeading;
      /*  System.out.println("SHOOTER_Turnto Current  " + Math.toDegrees(currentHeading));
        System.out.println("SHOOTER_Turnto Target  " + Math.toDegrees(targetAngle));
        System.out.println("SHOOTER_Turnto Turn  " + Math.toDegrees(normAngle));*/
        robot.drive.turnAsync(normAngle);
        //robot.drive.turnAsync(targetAngle);
    }
    public void turnTo(double targetAngle)
    {
        turnTo(targetAngle, true);
    }

    void handlePowershot()
    {


        switch (endGame) {
            case START:
                robot.drive.followTrajectoryAsync(powerTraj);
                if (!robot.shooter.isShooterOn) {
                    targetVelocity = robot.shooter.shooterOn();
                    System.out.println("SHOOTER ON target vel " + targetVelocity);
                }
                endGame=powershotState.TRAJECTORY_1;
                break;

            //TODO: CHANGE ORDER FOR OUTSIDE RED
            case TRAJECTORY_1:

                if (!robot.drive.isBusy()) {
                    System.out.println("SHOOTER_FIRSTTURN_X "+robot.drive.getPoseEstimate().getX());
                    System.out.println("SHOOTER_FIRSTTURN_Y "+ robot.drive.getPoseEstimate().getY());
                    turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1)-Math.toRadians(5));
                    endGame=powershotState.FIRST_TURN;
                    System.out.println("SHOOTER_firstAngle " + Math.toDegrees(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1)));
                }
                break;

            case FIRST_TURN:
               // System.out.println("SHOOTER Waiting for turn");
                if (!robot.drive.isBusy()){
                    System.out.println("SHOOTER Turn done moving to on");
                    endGame=powershotState.SHOOTER_ON;
                }
                break;

            case SHOOTER_ON:
                // System.out.println("SHOOTER_shooterOn");
                System.out.println("SHOOTER Waiting for target vel");
                if (robot.shooter.isShooterReady(targetVelocity)) {
                    System.out.println("SHOOTER target vel " + targetVelocity);
                    waitTimer1.reset();
                    endGame=powershotState.SHOOT;
                } //will need to add a timer later to move on in case we never get up to speed
                break;

            case SHOOT:
                boolean done;

                // System.out.println("SHOOTER_shootInState");
                // System.out.println("SHOOTER_shootInState still turning " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
                if (!robot.drive.isBusy()&& waitTimer1.milliseconds()>1000) {//making sure our turn is done  && waitTimer1.time()>1500
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
                        //   System.out.println("SHOOT_Shooter Ready " + robot.shooter.shooter.getVelocity());
                        //   System.out.println("SHOOT_Shooter Ready " + robot.shooter.isShooterReady(targetVelocity));
                        robot.shooter.pusherIn();
                        shootCount += 1;
                        waitTimer1.reset();
                    }
                    if (shootCount < 3) {
                        //System.out.println("SHOOTER_shootToTurn");

                        endGame=powershotState.TURN;
                    }
                    else
                    {
                        if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 1000) {

                            currentMode = Mode.NORMAL_CONTROL;
                            shooterMode=Shooter_State.SHOOTER_RESET;
                            endGame=powershotState.IDLE;
                        }
                    }
                }
                break;

            case TURN:
                // System.out.println("SHOOTER_turnInState");
                if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 1000 ) {
                    System.out.println("SHOOTER_ringShot");
                    robot.shooter.pusherOut();
                    powerTurn();
                    endGame=powershotState.SHOOTER_ON;
                }
                break;

            case IDLE:
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
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot2)-POWEROFFSET2);
            }
            else
            {
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


package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAlignWithPoint;
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
    private boolean intakeButtonDown;
    private boolean wobbleButtonDown;
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

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT,
        RESET_ODOMETRY
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
        WOBBLE_OPEN
    }

    enum Shooter_State {
        SHOOTER_ON,
        RAMP_UP,
        SHOOT,
        PUSHER_OUT,
        SHOOTER_OFF,
        RESET_SHOOT_COUNT
    }

    double targetVelocity = 2000;
    private Intake_State intakeMode = Intake_State.INTAKE_OFF;
    private Mode currentMode = Mode.NORMAL_CONTROL;
    private Shooter_State shooterMode = Shooter_State.SHOOTER_OFF;
    private Wobble_State wobbleMode = Wobble_State.WOBBLE_DOWN;
    private int shootCount = 0;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        robot = new Robot(hardwareMap, telemetry);

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
        int isRed = 1;
        Pose2d startPose = new Pose2d(-63, -24 * isRed, 0);
//        robot.drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        //isRed = PoseStorage.isRed;
        robot.drive.getLocalizer().setPoseEstimate(startPose);

        shooterMode = Shooter_State.SHOOTER_OFF;
        wobbleMode = Wobble_State.WOBBLE_UP;

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
                        currentMode = Mode.ALIGN_TO_POINT;
                    }

                    if (gamepad1.x && !resetOdomButtonDown) {
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
                    break;
                case ALIGN_TO_POINT:
                    // Switch back into normal driver control mode if `rb` is pressed
                    if (gamepad1.right_bumper && !driveButtonDown) {
                        driveButtonDown = true;
                        currentMode = Mode.NORMAL_CONTROL;
                    }


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
                    Pose2d newPose = new Pose2d(-63, -63 * isRed, 0);
                    robot.drive.getLocalizer().setPoseEstimate(newPose);
                    currentMode = Mode.NORMAL_CONTROL;
                    break;
            }

            handleIntake();
            handleWobble();
            handleChangeHeading();

            switch (shooterMode) {

                case SHOOTER_OFF:
                    shootCount = 0;
                    robot.shooter.shooterOff();
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
                    }
                    if (debug) System.out.println("SHOOT_Shoot Count " + shootCount);
                    if (debug) System.out.println("SHOOT_Shoot Number " + shootNumber);
                    if (shootCount <= shootNumber) {

                        if (robot.shooter.isShooterReady(targetVelocity)) {
                            if (debug)
                                System.out.println("SHOOT_Shooter Ready " + robot.shooter.shooter.getVelocity());
                            if (debug)
                                System.out.println("SHOOT_Shooter Ready " + robot.shooter.isShooterReady(targetVelocity));
                            if (!robot.shooter.shooting) {
                                shootCount += 1;
                                shooterMode = Shooter_State.SHOOT;
                                if (debug) System.out.println("SHOOT_Shooting");
                            }
                        }
                    } else {
                        shooterMode = Shooter_State.SHOOTER_OFF;
                        shootNumber = 3;
                        currentMode = Mode.NORMAL_CONTROL;
                        robot.shooter.pusherOut();
                        if (debug) System.out.println("SHOOT_Done");
                    }
                    break;

                case SHOOT:
                    robot.shooter.pusherIn();
                    shooterMode = Shooter_State.RAMP_UP;
                    if (debug) System.out.println("SHOOT_Done");
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

            robot.drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading
            headingController.update(poseEstimate.getHeading());

            // Update he localizer
            robot.drive.getLocalizer().update();


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

            telemetry.addData("heading", poseEstimate.getHeading());
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
        if (!gamepad1.x) resetOdomButtonDown=false;

        if (!gamepad2.x) shootButtonDown=false;
        if (!gamepad2.b) powerShotButtonDown=false;
        if (!gamepad2.dpad_down) manualShooterDecreaseButtonDown=false;
        if (!gamepad2.dpad_up) manualShooterIncreaseButtonDown=false;
        if (!gamepad2.right_bumper) ringIncreaseButtonDown=false;
        if (!gamepad2.left_bumper) ringDecreaseButtonDown=false;
        if (!gamepad2.dpad_right) psIncreaseButtonDown=false;
        if (!gamepad2.dpad_left) psDecreaseButtonDown=false;
        if (!gamepad1.dpad_right) headingRightButtonDown=false;
        if (!gamepad1.dpad_left) headingLeftButtonDown=false;
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
                if (gamepad1.b && !wobbleButtonDown){
                    robot.wobble.lowerWobbleFromFront();
                    wobbleMode=Wobble_State.WOBBLE_DOWNWAIT;
                }
                break;
            case WOBBLE_DOWNWAIT:
                //if (robot.wobble.isWobbleDown())
                //{
                        wobbleMode = Wobble_State.WOBBLE_OPEN;
                //}
                break;
            case WOBBLE_OPEN:
                robot.wobble.openClaw();
                wobbleMode=Wobble_State.WOBBLE_UP;
                break;
            case WOBBLE_CLOSE:
                if (gamepad1.b && !wobbleButtonDown){
                    robot.wobble.closeClaw();
                    wobbleMode=Wobble_State.WOBBLE_UP;
                    wobbleWait.reset();
                }
                break;
            case WOBBLE_UP:
                if (wobbleWait.time() > 500)
                {
                    //robot.wobble.raiseWobble();
                    wobbleMode = Wobble_State.WOBBLE_DOWN;
                }
                break;
        }
    }
}
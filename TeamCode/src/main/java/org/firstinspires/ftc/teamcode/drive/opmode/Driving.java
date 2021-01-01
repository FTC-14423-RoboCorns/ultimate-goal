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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Shooter;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
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

    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }
    enum Shooter_State {
        SHOOTER_ON,
        RAMP_UP,
        SHOOT,
        PUSHER_OUT,
        SHOOTER_OFF
    }

    double targetVelocity = 1900;

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private Shooter_State shooterMode = Shooter_State.SHOOTER_OFF;
    private int shootCount = 0;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        Robot robot = new Robot(hardwareMap, telemetry);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        int isRed = 1;
        Pose2d startPose = new Pose2d(-63, -24 * isRed, 0);
//        robot.drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        robot.drive.getLocalizer().setPoseEstimate(startPose);

        shooterMode = Shooter_State.SHOOTER_OFF;

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
        Vector2d targetPosition = new Vector2d(robot.shooter.redGoal.x, robot.shooter.redGoal.y);
        robot.shooter.currentTarget = robot.shooter.redGoal;
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

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if `a` is pressed
                    if (gamepad1.a) {
                        currentMode = Mode.ALIGN_TO_POINT;
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
                    // Switch back into normal driver control mode if `b` is pressed
                    if (gamepad1.b) {
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
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
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
            }



            switch (shooterMode) {

                case SHOOTER_OFF:
                    shootCount = 0;
                    robot.shooter.shooterOff();
                    if (gamepad1.y)
                    {
                        shooterMode = Shooter_State.SHOOTER_ON;
                    }
                    break;

                case SHOOTER_ON:
                    if (!robot.shooter.isShooterOn) {
                        targetVelocity = robot.shooter.shooterOn();
                    }
                    shooterMode = Shooter_State.RAMP_UP;
                    break;

                case RAMP_UP:
                if (!robot.shooter.isShooterReady(targetVelocity)) {
                    robot.shooter.pusherOut();
                }

                if (shootCount < 3)
                {
                    if (robot.shooter.isShooterReady(targetVelocity)) {
                        shooterMode = Shooter_State.SHOOT;
                        shootCount += 1;
                    }
                }
                else
                {
                    shooterMode = Shooter_State.SHOOTER_OFF;
                }
                break;

                case SHOOT:
                        robot.shooter.pusherIn();
                        shooterMode = Shooter_State.RAMP_UP;
                break;
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
            telemetry.addData("angle", robot.shooter.angle);
            telemetry.addData("shooter Height", robot.shooter.shooterHeight);
            telemetry.addData("crank Angle", robot.shooter.crankAngle);
            telemetry.addData("lift Pos", robot.shooter.liftPos);
            telemetry.addData("lift Pos", shooterMode);

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}

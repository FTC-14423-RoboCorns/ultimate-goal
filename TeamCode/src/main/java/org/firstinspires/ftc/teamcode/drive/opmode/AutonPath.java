package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.drive.Robot;



public class AutonPath {
    private Robot robot;


    private static double INSIDEX=-63.5;//-63
    private static double INSIDEY=-24;
    public int isRed=1;
private boolean debug=true;
    public enum CurrentTarget {
        RED_GOAL,
        RED_POWERSHOT,
        BLUE_GOAL,
        BLUE_POWERSHOT
    }

public    enum StartPosEnum {
        INSIDE_RED,
        OUTSIDE_RED,
        INSIDE_BLUE,
        OUTSIDE_BLUE
    }


public    StartPosEnum startPos;
public CurrentTarget currentTarget;
public CurrentTarget[] currentTargetArray= new CurrentTarget[3];
    public int ringPosition=2;
    public double firstAngle;
    public double[] firstAngleArray=new double[3];
    public AutonPath(Robot theRobot){
        robot=theRobot;
    }

    public Pose2d startPose;
    public enum PowershotTurnMode {
        TURN,
        STRAFE
    }
    public Trajectory trajectory1,trajectory2,trajectory3,pickUpRing, trajectory4,trajectory5,misswobble,strafe1,strafe2,ring2,ring3,drop1;

    public Trajectory[] trajectory1Array=new Trajectory[3];
            public Trajectory[] trajectory2Array=new Trajectory[3];
    public Trajectory[] trajectory3Array=new Trajectory[3];
    public Trajectory[] pickUpRingArray=new Trajectory[3];
    public Trajectory[] trajectory4Array=new Trajectory[3];
    public Trajectory[] trajectory5Array=new Trajectory[3];
    public Trajectory[] misswobbleArray=new Trajectory[3];
    public Trajectory[] strafe1Array=new Trajectory[3];
    public Trajectory[] strafe2Array=new Trajectory[3];
    public Trajectory[] ring2Array=new Trajectory[3];
    public Trajectory[] ring3Array=new Trajectory[3];
    public Trajectory[] drop1Array=new Trajectory[3];


    public PowershotTurnMode powershotTurnMode;
    public Vector2d firstShot;
    public Vector2d[] firstShotArray =new Vector2d[3];

    public void setCurrentTarget(CurrentTarget target){
        currentTarget=target;
        switch(target) {
            case RED_GOAL:
                robot.shooter.currentTarget=robot.shooter.redGoal;
                break;
            case RED_POWERSHOT:
                robot.shooter.currentTarget=robot.shooter.redPowerShot1;
                break;
            case BLUE_GOAL:
                robot.shooter.currentTarget=robot.shooter.blueGoal;
                break;
            case BLUE_POWERSHOT:
                robot.shooter.currentTarget=robot.shooter.bluePowerShot1;
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


    public void setStartPos(StartPosEnum startPos){
        switch (startPos){
            case INSIDE_RED:

                isRed = 1;
                //startPose = new Pose2d(-63, -24 * isRed, 0);
                startPose = new Pose2d(INSIDEX, INSIDEY * isRed, 0);
                //PowerTarget =363;

                break;

            case INSIDE_BLUE:

                isRed = -1;
                startPose = new Pose2d(INSIDEX, INSIDEY * isRed, 0);
                //PowerTarget =1;
                break;

        }
        robot.drive.setPoseEstimate(startPose);
    }



    public Trajectory setFirstTrajectory(CurrentTarget curTar, Vector2d first){
        switch (curTar) {
            case RED_POWERSHOT:
                //firstAngle=robot.shooter.angleToGoal(firstShot, robot.shooter.redPowerShot1);
                firstAngle=robot.shooter.angleToGoal(first, robot.shooter.redPowerShot1);//0
            return robot.drive.trajectoryBuilder(startPose)

                    //.splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), Math.toRadians(-15))
                    //.lineToLinearHeading(new Pose2d(firstShot, 0))
                    .lineToLinearHeading(new Pose2d(first, firstAngle))
                    .build();
             //affects lift height - change if we shoot for goal


            case RED_GOAL:
                firstAngle=robot.shooter.angleToGoal(first, robot.shooter.redGoal)-AutonShooting.POWEROFFSET;
                return  robot.drive.trajectoryBuilder(startPose)
                        //  .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), Math.toRadians(-15))
                        // .splineToLinearHeading(new Pose2d(-7, isRed * -24.5, 0), Math.toRadians(-15))
                        //.lineToLinearHeading(new Pose2d(shootX, isRed * shootY, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(first, firstAngle))
                        //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
                        .build();
                //robot.shooter.currentTarget=robot.shooter.redPowerShot1; //affects lift height - change if we shoot for goal

            default:
                return robot.drive.trajectoryBuilder(startPose)
                        //  .splineToLinearHeading(new Pose2d(-7, isRed * -12, robot.shooter.angleToGoal(-7, -12, robot.shooter.redPowerShot1)-POWEROFFSET), Math.toRadians(-15))
                        // .splineToLinearHeading(new Pose2d(-7, isRed * -24.5, 0), Math.toRadians(-15))
                        //.lineToLinearHeading(new Pose2d(shootX, isRed * shootY, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(first, firstAngle))
                        //.splineToLinearHeading(new Pose2d(-7, isRed * -12, Math.toRadians(PowerTarget)), 0)
                        .build();

        }
    }

    public Trajectory setFirstTrajectory(){
        return setFirstTrajectory(currentTarget,firstShot);
    }

    public void aimFirst(double angle,CurrentTarget tar) {
        switch (tar) {
            case RED_POWERSHOT:
                angle=0;
                turnTo(angle);
               // if (debug) System.out.println("SHOOTER_firstAngle " + Math.toDegrees(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot1)));
                if (debug) System.out.println("SHOOTER_firstAngle " + angle);
                //turnTo(Math.toRadians(0));
                break;
            case RED_GOAL:
                turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redGoal) - AutonShooting.POWEROFFSET);
                if (debug) System.out.println("SHOOTER_firstAngle " + Math.toDegrees(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(),  robot.shooter.redGoal) - AutonShooting.POWEROFFSET));
                break;
        }
    }

    public void aimFirst(){
        aimFirst(firstAngle,currentTarget);
    }

    }



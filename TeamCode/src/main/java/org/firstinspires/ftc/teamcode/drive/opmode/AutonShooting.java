package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Shooter;

public class AutonShooting {

    private boolean isBusy=false;
    private Robot robot;
    private AutonPath autonPath;
    public int desiredVelocity=0;
    public double targetVelocity=1950;
    ElapsedTime waitTimer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
boolean debug=false;

    public static final double POWEROFFSET = Math.toRadians(3)*-1;
    public static final double POWEROFFSET2 = 0;
    public static final double POWEROFFSET3 = 0;

private int shootCount;

    public enum ShootingState {
        IDLE,
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
        GO_SHOOT,
        GRAB_WOBBLE_2,
        RELOAD_WAIT,
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

    public ShootingState shootingState;


    public AutonShooting (Robot robot, AutonPath autonPath) {
        robot=robot;
        autonPath=autonPath;
        pusherOut();
    }

    public boolean isBusy(){
        return isBusy;
    }

    public void pusherIn(){
        robot.shooter.pusherIn();
    }

    public void pusherOut(){
        robot.shooter.pusherOut();
    }

    public void setCurrentTarget(Shooter.target currentTarget){
        robot.shooter.currentTarget=currentTarget;
    }

    public void shooterOn(){
        if (desiredVelocity==0) targetVelocity= robot.shooter.shooterOn();
        else targetVelocity= robot.shooter.shooterOn(desiredVelocity);
    }

    int secondShootTotal=0;
    //TODO: state for pushring



    public void handleShooting(){

        switch (shootingState){
        case IDLE:
            break;
        case SHOOTER_ON:
            isBusy=true;
        // System.out.println("SHOOTER_shooterOn");
        if (!robot.shooter.isShooterOn) {
            shooterOn();
        }
        if (robot.shooter.isShooterReady(targetVelocity)) {

            shootingState = ShootingState.SHOOT;
        } //will need to add a timer later to move on in case we never get up to speed
        break;

        case SHOOT:
        boolean done;
                /*if (debug) {
               // System.out.println("SHOOTER_shootInState");
               // System.out.println("SHOOTER_shootInState still turning " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
               }
                 */
        if (!robot.drive.isBusy()&& waitTimer1.time()>500) {//500//making sure our turn is done  && waitTimer1.time()>1500
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
                pusherIn();
                shootCount += 1;
                waitTimer1.reset();
            }

            if (shootCount < 3) {
                //System.out.println("SHOOTER_shootToTurn");

                shootingState = ShootingState.TURN;
            }
            else
            {
                if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 400) {//500
                    robot.shooter.shooterOff();
                    pusherOut();
                    isBusy=false;
                    shootingState=ShootingState.IDLE;
                }
            }
        }
        break;

        case TURN:
        // System.out.println("SHOOTER_turnInState");
        if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 400 ) {//500
            if (debug) {
                System.out.println("SHOOTER_ringShot");
            }
            robot.shooter.pusherOut();
            if (autonPath.currentTarget== AutonPath.CurrentTarget.RED_POWERSHOT||autonPath.currentTarget== AutonPath.CurrentTarget.BLUE_POWERSHOT){
             powerTurn();}
            shootingState = ShootingState.TURN2;
        }
        break;

        case TURN2:
        if (!robot.drive.isBusy()) {
            //if (ringPosition<2) turnTo(0);
            shootingState = ShootingState.SHOOTER_ON;
        }
        break;
            case GO_SHOOT:
                //if (!robot.drive.isBusy()) {
                //robot.intake.turnOn();
                isBusy=true;
                shootCount=0;
                autonPath.setCurrentTarget(AutonPath.CurrentTarget.RED_GOAL);
                //robot.shooter.currentTarget=robot.shooter.redGoal;
                if(autonPath.ringPosition ==1 )
                {
                    secondShootTotal =1;
                }
                else
                {
                    secondShootTotal =3;
                }
                shootingState = ShootingState.SHOOT_SECOND_BATCH;
                //}
                break;

            case SHOOT_SECOND_BATCH:

                // System.out.println("SHOOTER_shooterSecond");
                if (!robot.drive.isBusy()) {
                    waitTimer1.reset();
                    robot.intake.turnOff();

                    if (robot.shooter.isShooterReady(targetVelocity)) {

                        shootingState = ShootingState.SHOOTAGAIN;
                    } //will need to add a timer later to move on in case we never get up to speed
                }
                break;


            case SHOOTAGAIN:
                // boolean done;
                /*if (debug) {
               // System.out.prifntln("SHOOTER_shootInState");
               // System.out.println("SHOOTER_shootInState still turning " + Math.toDegrees(robot.drive.getPoseEstimate().getHeading()));
               }
                 */
                if (!robot.drive.isBusy()&& waitTimer1.milliseconds()>500) {//500//making sure our turn is done  && waitTimer1.time()>1500
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
                        shootingState = ShootingState.RELOADAGAIN;
                    }
                    else
                    {
                        if (!robot.shooter.isShooterReady(targetVelocity-200) || waitTimer1.time() >= 500) {
                            robot.shooter.shooterOff();
                            pusherOut();
                            isBusy=false;
                            shootingState = ShootingState.IDLE;
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
                    shootingState = ShootingState.SHOOT_SECOND_BATCH;
                }
                break;


        }
        robot.shooter.update(robot.drive.getPoseEstimate());
    }



    public void powerTurn()
    {
        // PowerTarget=PowerTarget - (isRed*6);
        //turnTo(isRed*-6);

        if(autonPath.isRed == 1)
        {
            if(shootCount == 1) {
                switch (autonPath.powershotTurnMode) {
                    case STRAFE:
                    autonPath.strafe1 = robot.drive.trajectoryBuilder(autonPath.trajectory1.end())
                            //.strafeLeft(8)
                            .lineToLinearHeading(new Pose2d(autonPath.firstShot.getX(), autonPath.firstShot.getY() + 8, 0))
                            .build();
                    robot.drive.followTrajectoryAsync(autonPath.strafe1);
                    break;
                    case TURN:
                    autonPath.turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot2)-POWEROFFSET2);
                    break;
                }
            }
            else
            {
                switch (autonPath.powershotTurnMode) {
                    case STRAFE:
                autonPath.strafe2 = robot.drive.trajectoryBuilder(autonPath.strafe1.end())
                        //.strafeLeft(8.5)
                        .lineToLinearHeading(new Pose2d(autonPath.firstShot.getX(),autonPath.firstShot.getY()+15,0))//was 14 for ringposition==0
                        .build();
                robot.drive.followTrajectoryAsync(autonPath.strafe2);
                break;
                    case TURN:
                autonPath.turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.redPowerShot3)-POWEROFFSET3);
            break;}

                }
        }
        else
        {//TODO: Make blue shooting turn code
            if(shootCount == 1)
            {
                autonPath.turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.bluePowerShot2));
            }
            else
            {
                autonPath.turnTo(robot.shooter.angleToGoal(robot.drive.getPoseEstimate().getX(), robot.drive.getPoseEstimate().getY(), robot.shooter.bluePowerShot3));
            }
        }
    }



}

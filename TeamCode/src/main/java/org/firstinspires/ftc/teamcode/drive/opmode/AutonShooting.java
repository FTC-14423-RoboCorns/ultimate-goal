package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Shooter;

public class AutonShooting {

    private boolean isBusy=false;
    private Robot robot;
    public int desiredVelocity=0;
    public double targetVelocity=1950;
    ElapsedTime waitTimer1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
boolean debug=false;



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


    public AutonShooting (Robot robot) {
        robot=robot;
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
            if (ringPosition<1) powerTurn();
            shootingState = ShootingState.TURN2;
        }
        break;

        case TURN2:
        if (!robot.drive.isBusy()) {
            //if (ringPosition<2) turnTo(0);
            shootingState = ShootingState.SHOOTER_ON;
        }
        break;

    }}


}

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;

public class AutonWobble {
    private Robot robot;
    enum WobbleState {
        WOBBLE_RAISE,
        WOBBLE_RAISEWAIT,
        WOBBLE_LOWER,
        WOBBLE_LOWERWAIT,
        WOBBLE_OFF
    }
    WobbleState wobbleState;
    ElapsedTime wobbleWait = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public int wobblePos = 0;
    public AutonWobble (Robot theRobot) {
        robot=theRobot;
        closeClaw();
        wobbleState=WobbleState.WOBBLE_RAISE;
    }

    public void closeClaw(){
        robot.wobble.closeClaw();
    }

    public void openClaw(){
        robot.wobble.openClaw();
    }

    public void setWobblePos(int pos){
        wobblePos=pos;
        wobbleState = WobbleState.WOBBLE_RAISE;
    }

    public boolean isBusy() {
        return !robot.wobble.isWobbleThere(wobblePos);
    }

    public void handleWobble() {
        switch (wobbleState)
        {
            case WOBBLE_RAISE:
                // System.out.println("WOBBLE_raiseInState");
                //if(!robot.wobble.isWobbleUp(wobblePos))
                //robot.wobble.wobbleSetRaise(wobblePos); //shouldn't need
                robot.wobble.wobbleMovetoPosition(wobblePos);
                /*
                if(!robot.wobble.isWobbleThere(wobblePos))
                {
                    wobbleWait.reset();
                  //  robot.wobble.raiseWobbleFromFront();
                    robot.wobble.wobbleMovetoPosition(wobblePos);
                    wobbleState = WobbleState.WOBBLE_RAISEWAIT;
                   // System.out.println("WOBBLE_raise " + wobblePos);
                }*/

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

}

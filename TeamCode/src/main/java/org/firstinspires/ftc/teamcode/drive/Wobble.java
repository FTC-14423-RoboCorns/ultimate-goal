package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Wobble
{
    public DcMotorEx wobble;
    public Servo wobbleServo;
    public boolean on;
    private static int START_POSITION = 0;
    private static int STOP_POSITION= 1140;
    private static double SERVO_OPEN = 0;
    private static double SERVO_CLOSED =0.24;// 0.23;
    private boolean raise =true;
    public boolean override=false;

    public Wobble(HardwareMap hardwareMap)
    {
        Init(hardwareMap);
    }

    public void Init(HardwareMap hardwareMap)
    {
        wobble = hardwareMap.get(DcMotorEx.class, "Wobble");
        wobble.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobble.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wobble.setDirection(DcMotorEx.Direction.REVERSE);

        //TODO: set position that wobble starts in
        //wobble.setTargetPosition();

        wobbleServo = hardwareMap.get(Servo.class, "WobbleServo");
        //TODO: set a postion that closes the wobble servo
        //wobbleServo.setPosition();
        closeClaw();
        on=false;



    }
    //Opens Wobble Hand and releases Wobble
    public void openClaw() {
        //TODO: set a postion that opens the wobble servo
        wobbleServo.setPosition(SERVO_OPEN);
    }

    //Closes Wobble Hand and grabs Wobble
    public void closeClaw() {
        //TODO: set a postion that closes the wobble servo
        wobbleServo.setPosition(SERVO_CLOSED);
    }

    //Moves Wobble Arm to Raised Position (over Ramp)
    public void raiseWobbleFromFront()
    {
        //TODO: set position that raises the wobble
        if(wobble.getCurrentPosition()<(STOP_POSITION-50))
        {
            wobble.setTargetPosition(wobble.getCurrentPosition()+50);
            wobble.setPower(1);
            wobble.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            on=true;
        }
    }

    public void fastMovetoPos(int pos)
    {
        int newPos;
        if (!override) newPos = Range.clip(pos,START_POSITION,STOP_POSITION);
        else  newPos=pos;
        if (!isWobbleThere(newPos)){
        wobble.setTargetPosition(newPos);
        wobble.setPower(.5);
        wobble.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        on=true;}
    }

    //Moves Wobble Arm to Lowered Position (Extended position)
    public void lowerWobbleFromFront()
    {
        //TODO: set position that lowers the wobble
        if(wobble.getCurrentPosition()>(START_POSITION+50)||override)
        {
            wobble.setTargetPosition(wobble.getCurrentPosition()-50);
            wobble.setPower(1);
            wobble.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            on=true;
        }
    }

    public void wobbleSetRaise(double newPos){
        double curPos = wobble.getCurrentPosition();
        if (newPos>=curPos){
            raise=true;
        } else {
            raise=false;
        }
    }

    public void wobbleMovetoPosition(int newPos){
        /*double curPos = wobble.getCurrentPosition();
        if (newPos>=curPos){
            raiseWobbleFromFront();
            raise=true;
        } else {
            lowerWobbleFromFront();
            raise=false;
        }

         */
        fastMovetoPos(newPos);
    }

    public void wobbleOff()
    {
        wobble.setPower(0);
        on=false;
    }

    public boolean isWobbleUp(int pos) {
        if (wobble.getCurrentPosition() >= (pos - 30)) {
            return true;
        } else return false;
    }

    public void resetWobble(){
        wobble.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        override=false;
    }
    public boolean isWobbleDown (int pos) {
        if (wobble.getCurrentPosition() <= (pos + 30)) {
              return true;
           } else return false;
        }
    public boolean isWobbleThere(int pos){
        double curPos = wobble.getCurrentPosition();
        /*if (raise){
            return isWobbleUp(pos);
        } else
            return isWobbleDown(pos);

         */
        double lower = pos -30;
        double upper= pos + 30;
        if ((curPos > lower) && (curPos < upper)) {
            return true;
        } else {
            return false;
        }
    }
}

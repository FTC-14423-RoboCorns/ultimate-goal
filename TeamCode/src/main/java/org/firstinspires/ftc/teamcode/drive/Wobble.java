package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble
{
    public DcMotorEx wobble;
    public Servo wobbleServo;
    public boolean on;
    private static int START_POSITION = 0;
    private static int STOP_POSITION= 1140;
    private static double SERVO_OPEN = 1;
    private static double SERVO_CLOSED = 0.55;


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
            wobble.setTargetPosition(wobble.getCurrentPosition()+30);
            wobble.setPower(1);
            wobble.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            on=true;
        }
    }

    //Moves Wobble Arm to Lowered Position (Extended position)
    public void lowerWobbleFromFront()
    {
        //TODO: set position that lowers the wobble
        if(wobble.getCurrentPosition()>(START_POSITION+50))
        {
            wobble.setTargetPosition(wobble.getCurrentPosition()-30);
            wobble.setPower(1);
            wobble.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            on=true;
        }
    }

    public void wobbleOff()
    {
        wobble.setPower(0);
        on=false;
    }

    public boolean isWobbleUp(int pos) {
        if (wobble.getCurrentPosition() >= (pos)) {
            return true;
        } else return false;
    }

    public boolean isWobbleDown (int pos) {
        if (wobble.getCurrentPosition() <= (pos)) {
              return true;
           } else return false;
        }

}

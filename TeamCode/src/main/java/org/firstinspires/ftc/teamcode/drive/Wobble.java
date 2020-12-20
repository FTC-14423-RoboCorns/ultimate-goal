package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble
{
    public DcMotorEx wobble;
    public Servo wobbleServo;

    public Wobble(HardwareMap hardwareMap)
    {
        Init(hardwareMap);
    }

    public void Init(HardwareMap hardwareMap)
    {
        wobble = hardwareMap.get(DcMotorEx.class, "Wobble");
        wobble.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobble.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //TODO: set position that wobble starts in
        //wobble.setTargetPosition();

        wobbleServo = hardwareMap.get(Servo.class, "WobbleServo");
        //TODO: set a postion that closes the wobble servo
        //wobbleServo.setPosition();

    }

    //Opens Wobble Hand and releases Wobble
    public void placeWobble() {
        //TODO: set a postion that opens the wobble servo
        //wobbleServo.setPosition();
    }

    //Closes Wobble Hand and grabs Wobble
    public void getWobble() {
        //TODO: set a postion that closes the wobble servo
        //wobbleServo.setPosition();
    }

    //Moves Wobble Arm to Raised Position
    public void raiseWobble()
    {
        //TODO: set position that raises the wobble
        //wobble.setTargetPosition();
    }

    //Moves Wobble Arm to Lowered Position
    public void lowerWobble()
    {
        //TODO: set position that lowers the wobble
        //wobble.setTargetPosition();
    }
}

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.ftclib.FtcPixyCam2;

public class Robot
{
    public DcMotorEx intake, shooter, wobble;
    public CRServo roller, magazine;
    public Servo wobbleServo, intakeServo;

    public FtcPixyCam2 pixy;
//TODO: add lift servo and mag
    public Robot(HardwareMap hardwareMap) {
        Init(hardwareMap);
    }

    public void Init(HardwareMap hardwareMap) {
        //lift = hardwareMap.get(DcMotorEx.class, "Lift");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        wobble = hardwareMap.get(DcMotorEx.class, "Wobble");

        roller = hardwareMap.get(CRServo.class, "Roller");
        magazine = hardwareMap.get(CRServo.class, "Magazine");

        wobbleServo = hardwareMap.get(Servo.class, "WobbleServo");
        intakeServo = hardwareMap.get(Servo.class, "IntakeServo");

        //TODO: Figure out address and bit for pixycam2
        //pixy = new FtcPixyCam2(hardwareMap, "PixyCam",);

    }

    public void resetEncoders() {
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobble.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobble.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
}

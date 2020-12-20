package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.ftclib.FtcPixyCam2;
import org.firstinspires.ftc.teamcode.drive.Wobble;

public class Robot
{
    public FtcPixyCam2 pixy;
    public Wobble wobble;
    public IntakeAndRamp intake;
    public Shooter shooter;

    public Robot(HardwareMap hardwareMap) {
        Init(hardwareMap);
    }

    public void Init(HardwareMap hardwareMap) {
        //TODO: Figure out address and bit for pixycam2
        //pixy = new FtcPixyCam2(hardwareMap, "PixyCam",);
        wobble = new Wobble(hardwareMap);
        intake = new IntakeAndRamp(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }
}

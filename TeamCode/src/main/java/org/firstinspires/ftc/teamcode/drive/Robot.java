package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Unused.ftclib.ftclib.FtcPixyCam2;

public class Robot
{
    public Wobble wobble;
    public IntakeAndRamp intake;
    public Shooter shooter;
    public SampleMecanumDrive drive;
    public PixyAnalog pixy;

    public Robot(HardwareMap hardwareMap) {
        Init(hardwareMap);
    }

    public void Init(HardwareMap hardwareMap) {
        //TODO: Figure out address and bit for pixycam2
        //pixy = new FtcPixyCam2(hardwareMap, "PixyCam",);
        //wobble = new Wobble(hardwareMap);
        //intake = new IntakeAndRamp(hardwareMap);
        shooter = new Shooter(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        pixy = new PixyAnalog(hardwareMap);
    }
}

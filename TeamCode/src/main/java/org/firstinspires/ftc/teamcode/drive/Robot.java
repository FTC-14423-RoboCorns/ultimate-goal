package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Unused.ftclib.ftclib.FtcPixyCam2;

public class Robot
{
    public Telemetry telemetry;
    public Wobble wobble;
    public IntakeAndRamp intake;
    public Shooter shooter;
    public SampleMecanumDrive drive;
    public PixyAnalog pixy;

    public Robot(HardwareMap hardwareMap, Telemetry telem) {
        Init(hardwareMap);
        this.telemetry = telem;
    }

    public void Init(HardwareMap hardwareMap) {
        //TODO: Figure out address and bit for pixycam2
        //pixy = new FtcPixyCam2(hardwareMap, "PixyCam",);
        //wobble = new Wobble(hardwareMap);
        //intake = new IntakeAndRamp(hardwareMap);
        shooter = new Shooter(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        pixy = new PixyAnalog(hardwareMap);
    }
}

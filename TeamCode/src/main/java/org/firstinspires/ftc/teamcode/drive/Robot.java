package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import Unused.ftclib.ftclib.FtcPixyCam2;

public class Robot
{
    public Telemetry telemetry;
    public Wobble wobble;
    public IntakeAndRamp intake;
    public Shooter shooter;
    public SampleMecanumDrive drive;
    public PixyAnalog pixy;
    public List<LynxModule> allHubs;
    public boolean imu;




    public Robot(HardwareMap hardwareMap, Telemetry telem,boolean imu) {
        allHubs=hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        this.telemetry = telem;
        this.imu=imu;
        Init(hardwareMap);

    }

    public void Init(HardwareMap hardwareMap) {

        wobble = new Wobble(hardwareMap);
        intake = new IntakeAndRamp(hardwareMap);
        shooter = new Shooter(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        if (imu)
        drive.USE_IMU=1;
        else
            drive.USE_IMU=0;
        pixy = new PixyAnalog(hardwareMap);

    }
}

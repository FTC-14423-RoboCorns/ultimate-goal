package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.trclib.TrcUtil;

import org.firstinspires.ftc.teamcode.ftclib.FtcPixyCam2;
import org.firstinspires.ftc.teamcode.trclib.TrcPixyCam2;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Demonstrates empty OpMode
 */
@Config
@TeleOp(name = "PixyCam", group = "Concept")
//@Disabled
public class PixyCamOpMode extends OpMode {

    FtcPixyCam2 pixy;

    private TrcPixyCam2.Block[] blocks;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        pixy = new FtcPixyCam2(hardwareMap, "PixyCam", 0x54, true);
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
        //pixy = new FtcPixyCam2(hardwareMap, "PixyCam", 0x54, false);
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start()
    {
        pixy.setEnabled(true);
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", pixy.isEnabled());
        //telemetry.addData("Hardware Version", pixy.getHardwareVersion());
        telemetry.addData("Res Height", pixy.getResolutionHeight());
        blocks = pixy.getBlocks((byte) 1, (byte) 5);
        telemetry.addData("Status", "get blocks");

        if(blocks != null) {
            telemetry.addData("Status", "found block");
            for(TrcPixyCam2.Block block : blocks) {
                telemetry.addData("BlockHeight", block.height);
                telemetry.addData("BlockWidth", block.width);
                telemetry.update();
            }
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
//bob is great. Bob for prez! I am Bob, and I approve this message. Bob 2024. message and data rates apply (your tax money goes to my auto loan debt, feed me)
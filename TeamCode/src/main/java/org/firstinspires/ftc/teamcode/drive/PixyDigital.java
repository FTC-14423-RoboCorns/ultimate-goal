package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PixyDigital
{



    private DigitalChannel  pixy;

    public PixyDigital(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        pixy = hardwareMap.get(DigitalChannel .class, "PixyDigital");
        // set the digital channel to input.
        pixy.setMode(DigitalChannel.Mode.INPUT);
    }


    public boolean ringSensed()
    {
        return pixy.getState();
    }


}

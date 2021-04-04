package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class GyroAnalog
{
    private double rawAngle;
    private double finalAngle;
    private double breakAngle;
    private double lastAngle;
    private double tempAngle;
    public AnalogInput gyro;
    private double BREAK=3.216;//Arbitrarily high, need to find
    // private PixyDigital pixydig;
    public GyroAnalog(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        gyro = hardwareMap.get(AnalogInput.class, "Gyro");

    }
    public double readGyroVoltage() {
        return gyro.getVoltage();
    }
    public double readGyro()
    {
        rawAngle=gyro.getVoltage();
        if (rawAngle<BREAK) {
            finalAngle = 2*Math.PI - (2*Math.PI*rawAngle/3.3);
           // finalAngle = (2 * Math.PI * (3.227 - (rawAngle-.002))) / 3.3;//old range .003,3.225
        } else
        {
            //breakAngle= ((2 * Math.PI * (3.3 - (BREAK))) / 3.3);
            finalAngle= ((1-(BREAK/3.3))*2*Math.PI*(3.227-rawAngle))/(3.227-BREAK);
            //finalAngle=breakAngle+lastAngle;
        }
        //finalAngle=((2*Math.PI*(3.3-(rawAngle)))/3.3);//old range .003,3.225
        return finalAngle;

        /*let's say BREAK is 90 degrees (down to 0). BREAK=2.475
        zero should be 3.3, but is now 3.227
        If we get a 3.1 finalAngle= (1-(2.475/3.3))*2*PI*(3.227-3.1)/(3.227-2.475)
        =.265RAD = 15 degrees. This makes sense, because
        (3.227-3.1)=.127 and (3.227-2.475)=.752 and .127/.752=16.8% of the portion after the Break
        That portion is 90 degrees.16.8% of 90 is 15 degrees
         */
    }

}


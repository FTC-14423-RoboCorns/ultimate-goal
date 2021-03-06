package Unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import Unused.ftclib.ftclib.FtcI2cDeviceSynch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import Unused.hallib.hallib.HalDashboard;
@Disabled
@TeleOp(name="Test: FTC Raw Pixy Test", group="Test")
public class Pixy2Raw extends OpMode
{
    byte[] response;
    byte[] request = new byte[6];
    private HalDashboard dashboard;
    private FtcI2cDeviceSynch pixyCam;
    private static final short PIXY2_SEND_SYNC   = (short)0xc1ae;
    @Override
    public void init()
    {
        dashboard = HalDashboard.getInstance();
        pixyCam = hardwareMap.get(FtcI2cDeviceSynch.class, "PixyCam");
        pixyCam.setDeviceInfo(HardwareDevice.Manufacturer.Other, "Pixy Camera v2");
        pixyCam.setI2cAddress(0x54, true);
        pixyCam.getDeviceClient().engage();
       telemetry.addData("Data",pixyCam.getDeviceClient());



    }   //initRobot

    @Override
    public void loop()
    {
        request[0] = (byte)(PIXY2_SEND_SYNC & 0xff);
        request[1] = (byte)((PIXY2_SEND_SYNC >> 8) & 0xff);
        request[2] = (byte)32;
        request[3] = (byte)2;
        request[4]=(byte)1;
        request[5]=(byte)3;
        pixyCam.writeData(1,request,true);



        byte[] recvHeader =pixyCam.readData(1,6);
        byte[] recvData = recvHeader[3] > 0 ? pixyCam.readData(1, recvHeader[3]) : null;
        if (recvData != null)
        {
           response = new byte[recvHeader.length + recvData.length];
            System.arraycopy(recvHeader, 0, response, 0, recvHeader.length);
            System.arraycopy(recvData, 0, response, recvHeader.length, recvData.length);
        }
        else
        {
            response = recvHeader;
        }
        telemetry.addData("Data", response);


        /** byte[] data = pixyCam.readData(0,14);
         if (data != null)
         {
             for (int i = 0; i + 1 < data.length; i += 2)
             {
                 if (dashboard != null)
                     dashboard.displayPrintf(i/2 + 1, "%02d: %02x, %02x", i, data[i], data[i + 1]);
             }**/
        }
    }   //runPeriodic



   //FtcTestPixyCamRaw


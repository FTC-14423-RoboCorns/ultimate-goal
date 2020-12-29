package Unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import Unused.ftclib.ftclib.FtcI2cDeviceSynch;
import Unused.hallib.hallib.HalDashboard;
@Disabled
@TeleOp(name="Test: FTC Raw Test Pixy", group="Test")
public class Pixy2RawTest extends OpMode
{
    byte[] response;
    byte[] request = new byte[5];
    private HalDashboard dashboard;
    private FtcI2cDeviceSynch pixyCam;
    private static final short PIXY2_SEND_SYNC   = (short)0xc1ae;

    public class Version {
        public int hardwareVersion, firmwareBuild;
        public short majorFirmware, minorFirmware;
        public String firmwareType;

        private Version(byte[] payload) {
            hardwareVersion = payload[0];
            hardwareVersion |= payload[1] << 8;
            majorFirmware = payload[2];
            minorFirmware = payload[3];
            firmwareBuild = payload[4];
            firmwareBuild |= payload[5] << 8;
            byte[] firmwareString = new byte[payload.length - 6];
            System.arraycopy(payload, 6, firmwareString, 0, firmwareString.length);
            firmwareType = new String(firmwareString);
        }
    }
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
        //request[0] = (byte)(PIXY2_SEND_SYNC & 0xff);
        request[0] = (byte)((PIXY2_SEND_SYNC >> 8) & 0xff);
        request[1] = (byte)32;
        request[2] = (byte)2;
        request[3]=(byte)1;
        request[4]=(byte)3;
        pixyCam.writeData((byte)(PIXY2_SEND_SYNC & 0xff),request,true);



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


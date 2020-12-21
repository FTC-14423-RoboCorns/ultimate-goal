/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.ftclib;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.trclib.TrcDbgTrace;
import org.firstinspires.ftc.teamcode.trclib.TrcPixyCam2;
import org.firstinspires.ftc.teamcode.drive.NewPixy2;
import org.firstinspires.ftc.teamcode.drive.LynxI2CDeviceSynchEx;
import java.util.Arrays;

/**
 * This class implements a platform dependent pixy camera 2 that is connected to an I2C bus.
 * It provides access to the last detected objects reported by the pixy camera asynchronously.
 */
public class FtcPixyCam2Robo extends TrcPixyCam2
{
    private static final int DEF_I2C_ADDRESS = 0x54;
    private static final boolean USE_BUFFERED_READ = false;
    private final NewPixy2 pixyCam;
    private LynxI2CDeviceSynchEx deviceSynch;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param devAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcPixyCam2Robo(HardwareMap hardwareMap, String instanceName, int devAddress, boolean addressIs7Bit)
    {
        super(instanceName);
        pixyCam = new NewPixy2(hardwareMap.get(LynxI2CDeviceSynchEx.class, instanceName));
        deviceSynch=pixyCam.getDeviceClient();
       //mot needed pixyCam.deviceClient.setDeviceInfo(HardwareDevice.Manufacturer.Other, "Pixy Camera v2");

    }   //FtcPixyCam2

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param devAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcPixyCam2Robo(String instanceName, int devAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, devAddress, addressIs7Bit);
    }   //FtcPixyCam2

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcPixyCam2Robo(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, DEF_I2C_ADDRESS, true);
    }   //FtcPixyCam2

    /**
     * This method checks if the pixy camera is enabled.
     *
     * @return true if pixy camera is enabled, false otherwise.
     */

    public boolean isEnabled()
    {
        final String funcName = "isEnabled";
        boolean enabled = deviceSynch.isEngaged();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%b", enabled);
        }

        return enabled;
    }   //isEnable

    /**
     * This method enables/disables the pixy camera.
     *
     * @param enabled specifies true to enable pixy camera, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enanbled=%b", enabled);
        }
    if enabled {
        deviceSynch.engage();
    } else {
        deviceSynch.disengage();
    }
       // pixyCam.setEnabled(enabled);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    //
    // Implements TrcPixyCam2 abstract methods.
    //

    /**
     * This method issues an asynchronous read of the specified number of bytes from the device.
     */
    @Override
    public byte[] syncReadResponse()
    {
        final String funcName = "syncReadResponse";
        byte[] response;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);

        }
/*
        byte[] recvHeader = pixyCam.syncRead(-1, 6);
        byte[] recvData = recvHeader[3] > 0 ? pixyCam.syncRead(-1, recvHeader[3]) : null;

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
*/
        response = pixyCam.readPacket();
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "response: %s", Arrays.toString(response));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Arrays.toString(response));
        }

        return response;
    }   //syncReadResponse

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param data specifies the data buffer.
     */
    @Override
    public void syncWriteRequest(byte[] data)
    {
        final String funcName = "syncWriteRequest";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "data=%s", Arrays.toString(data));
        }

        pixyCam.writeComplete(data); //syncWrite(-1, data, data.length);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //syncWriteRequest

}   //class FtcPixyCam2
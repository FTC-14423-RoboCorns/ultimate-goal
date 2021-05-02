package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.689; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //12.6
    public static double LATERAL_DISTANCE =13.81; // in; distance between the left and right wheels (USED TO BE 14 - Dylan)
    public static double FORWARD_OFFSET = 2; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public static double XL_MULTIPLIER =1.0105156;//1.01694915;// 1.0059; //1.0163 was oldF
    public static double XR_MULTIPLIER =1.014061;//1.01694915;
    public static double Y_MULTIPLIER =1.021276;// 1.0095;
    private FtcDashboard dashboard;
    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LF"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RR"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RF"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
      //  frontEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        //rightEncoder.setDirection(Encoder.Direction.REVERSE);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("left", encoderTicksToInches(leftEncoder.getCurrentPosition()*XL_MULTIPLIER));
        packet.put("right", encoderTicksToInches(rightEncoder.getCurrentPosition()*XR_MULTIPLIER));
        packet.put("front actual", encoderTicksToInches(frontEncoder.getCurrentPosition()));
        packet.put("front correct", encoderTicksToInches(frontEncoder.getCurrentPosition()*Y_MULTIPLIER));
      //  dashboard.sendTelemetryPacket(packet);
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition())*XL_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition())*XR_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition())*Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * XL_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * XR_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}

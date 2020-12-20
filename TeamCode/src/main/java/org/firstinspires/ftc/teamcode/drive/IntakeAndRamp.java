package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Robot;

public class IntakeAndRamp {
    public DcMotorEx intake;
    public CRServo roller;
    public Servo intakeServo;

    public IntakeAndRamp(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap)
    {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        roller = hardwareMap.get(CRServo.class, "Roller");
        roller.setDirection(CRServo.Direction.FORWARD);

        //TODO: Set pos
        intakeServo = hardwareMap.get(Servo.class, "IntakeServo");
    }

    //Turns on the intake and ramp
    public void turnOn() {
        intake.setPower(1);
        roller.setPower(1);
    }

    //Turns off the intake and ramp
    public void turnOff() {
        intake.setPower(0);
        roller.setPower(0);
    }

    //Flips intake from start position to end position
    public void flipIntake() {
        //TODO: Change pos
        //intakeServo.setPosition();
    }
}

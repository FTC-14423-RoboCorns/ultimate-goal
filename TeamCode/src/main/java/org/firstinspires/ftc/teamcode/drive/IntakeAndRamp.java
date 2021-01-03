package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Robot;

public class IntakeAndRamp {
    public DcMotor intake;
    public CRServo roller;
    public Servo intakeServo;
    public boolean isIntakeOn;

    public IntakeAndRamp(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap)
    {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        roller = hardwareMap.get(CRServo.class, "Roller");
        roller.setDirection(CRServo.Direction.FORWARD);

        //TODO: Set pos
        //intakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        isIntakeOn=false;
    }

    //Turns on the intake and ramp
    public void turnOn() {
        intake.setPower(1);
        roller.setPower(1);
        isIntakeOn=true;
    }

    //Turns off the intake and ramp
    public void turnOff() {
        intake.setPower(0);
        roller.setPower(0);
        isIntakeOn=false;
    }
    public void spit() {
        intake.setPower(-1);
        roller.setPower(-1);
    }


    //Flips intake from start position to end position
    public void flipIntake() {
        //TODO: Change pos
        //intakeServo.setPosition();
    }
}

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
    public DcMotor wheels;
    public CRServo roller;
    public Servo intakeServo;
    public boolean isIntakeOn;
    public boolean isWheelOn;

    public IntakeAndRamp(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap)
    {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheels = hardwareMap.get(DcMotor.class, "Lift");
        wheels.setDirection(DcMotorSimple.Direction.REVERSE);
        wheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        wheels.setPower(1);
        isIntakeOn=true;
        isWheelOn=true;
    }

    public void wheelsOn() {
        wheels.setPower(1);
        isWheelOn=true;
    }
    public void wheelsOff() {
        wheels.setPower(0);
        isWheelOn=false;
    }

    //Turns off the intake and ramp
    public void turnOff() {
        intake.setPower(0);
        roller.setPower(0);
        wheels.setPower(0);
        isWheelOn=false;
        isIntakeOn=false;
    }
    public void spit() {
        intake.setPower(-1);
        roller.setPower(-1);
        wheels.setPower(-1);
        isWheelOn=true;
    }


    //Flips intake from start position to end position
    public void flipIntake() {
        //TODO: Change pos
        //intakeServo.setPosition();
    }
}

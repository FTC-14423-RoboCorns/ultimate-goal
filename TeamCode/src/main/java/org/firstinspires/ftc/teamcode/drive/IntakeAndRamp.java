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
    public Servo dropper;
    public Servo wingL;
    public Servo wingR;
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

        dropper = hardwareMap.get(Servo.class, "Dropper");

        wingL = hardwareMap.get(Servo.class, "wingL");
        wingR = hardwareMap.get(Servo.class, "wingR");
        wingInit();


        //TODO: Set pos
        //intakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        isIntakeOn=false;
    }

    //Turns on the intake and ramp
    public void lowerIntake(){
        dropper.setPosition(0);
    }
    public void raiseIntake(){
        dropper.setPosition(1);
    }
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

    public void wingInit(){
        wingL.setPosition(.27);
        wingR.setPosition(.78);
    }

    public void wingDown(){
        wingL.setPosition(.98);
        wingR.setPosition(.02);
    }

    public void wingUp(){
        wingL.setPosition(0.85);
        wingR.setPosition(0.15);
    }
    public void wingFullUp(){
        wingL.setPosition(.7);
        wingR.setPosition(.3);
    }


    public void spitwheels(){
            wheels.setPower(-1);
        }
}

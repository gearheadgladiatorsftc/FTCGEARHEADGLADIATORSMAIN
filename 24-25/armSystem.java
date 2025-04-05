package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//This is a constructor that allows easy use of the armsystem
public class ArmSystem {
    //Init variables
    public Servo shoulder;
    Servo elbow;
    DcMotorEx lift;

    Servo claw;
    Servo rotation;

   float INIT_CLAW = 0;
   float INIT_WRIST = 0;

    public ArmSystem(HardwareMap hardwareMap) { //init function, pass it the hardware map bc this is not accessible from this func
        //Init all the motors

        shoulder = hardwareMap.get(Servo.class, "shoulder");
        lift = hardwareMap.get(DcMotorEx.class, "lift");


        //shoulder.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);


        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw  = hardwareMap.get(Servo.class, "claw");
        elbow = hardwareMap.get(Servo.class, "elbow");
        rotation = hardwareMap.get(Servo.class, "rotation");
        //wrist = hardwareMap.get(Servo.class, "wrist");
        //wrist.setPosition(INIT_WRIST);
    }

    //The following are functions that can me called with Armsystem.funcion(param). Note the ArmSystem variable may be named something else.
    private double checkPower(double power) {
        if(Math.abs(power) > 1) {
            return power / Math.abs(power);
        } else {
            return power;
        }
    }
    public void setElbowPosition(double position) {
        elbow.setPosition(position);
    }

    public void setShoulderPower(double power) {
        shoulder.setPosition(Math.abs(power));
    }

    public void setLiftPower(double power) {
        lift.setPower(checkPower(power));
    }

    public void setClaw(double angle) {
        claw.setPosition(angle);
    }
    public void setRotation(double rot) { rotation.setPosition(rot); }

    //public void setWrist(double angle) {
        //wrist.setPosition(angle);
    //}

}

package org.firstinspires.ftc.teamcode;

//Allows for gamepad control of the wheels and armsystem

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.pidControllers.PIDBase;
@TeleOp(group = "drive")
public class MainTeleOp extends LinearOpMode {

    double clawClosedAngle = 0.0;
    double clawOpenAngle = 1.0;
    double rotationMid = 0.5;
    boolean FIELD_CENTRIC = false;
    float lim = 1;
    boolean readyState = false;


    ButtonEdgeDetector buttonA = new ButtonEdgeDetector();
    ButtonEdgeDetector buttonX = new ButtonEdgeDetector();

    ButtonEdgeDetector clawClosed = new ButtonEdgeDetector();
    ButtonEdgeDetector clawOpen = new ButtonEdgeDetector();
    DriveSystem driveSystem = null;
    ArmSystem armSystem = null;
    //hlController husky = new hlController(hardwareMap, telemetry);

    IMU imu = null;



    @Override
    public void runOpMode() throws InterruptedException { //idk what InterruptedException is, but it works!
        try {
            //telemetry.update();
            //Init constructors
            armSystem = new ArmSystem(hardwareMap);
            driveSystem = new DriveSystem(hardwareMap);
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward

            imu.initialize(parameters);

            waitForStart(); // Wait until start button is pressed


            long tic = System.nanoTime(); //System time in nanoseconds. Nanoseconds that have passed since last reboot
            //This is going to be the baseline

            while (!isStopRequested()) { // the ! reverses the input. So this evaluates to while(StopHasNotYetBeenRequested)

                driveControl(); //Execute tasks on driver 1 gamepad

                armControl(); //driver 2 gamepad
                long toc = System.nanoTime();
                if (toc - tic >= 30000000) {
                    if (gamepad2.dpad_up) {
                        armSystem.setShoulderPower(armSystem.shoulder.getPosition() + 0.01);
                    }
                    if (gamepad2.dpad_down) {
                        armSystem.setShoulderPower(armSystem.shoulder.getPosition() - 0.01);
                    }
                    if (gamepad2.dpad_right) {
                        armSystem.setShoulderPower(armSystem.shoulder.getPosition() + 0.003);
                    }
                    if (gamepad2.dpad_left) {
                        armSystem.setShoulderPower(armSystem.shoulder.getPosition() - 0.003);
                    }
                    if (gamepad2.y) {
                        armSystem.setElbowPosition(armSystem.elbow.getPosition() + 0.01);
                    }
                    if (gamepad2.a) {
                        armSystem.setElbowPosition(armSystem.elbow.getPosition() - 0.01);
                    }
                    if (gamepad2.b) {
                        armSystem.setElbowPosition(armSystem.elbow.getPosition() + 0.003);
                    }
                    if (gamepad2.x) {
                        armSystem.setElbowPosition(armSystem.elbow.getPosition() - 0.003);
                    }
                    tic = System.nanoTime();
                }


                telemetryUpdate();
            }
        }
        catch(Exception e){
            telemetry.addData("Message: ", e.getMessage());
            telemetry.addData("Cause: ", e.getCause());
        }
            

    }
    void driveButtonCheck(){
        //buttons
        if(gamepad1.options) {
            imu.resetYaw();
        }
        if(buttonA.updateActivate(gamepad1.a)) { //If a is pressed. This uses the ButtonEdgeDetector constructor
            FIELD_CENTRIC = !FIELD_CENTRIC;
        }
        if(gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.b){ // the || means or. If leftbumper or rightbumper or b
            lim = 4; //lim = limit of the motors. Slow down the bot
        }
        else if(armSystem.lift.getCurrentPosition() > 100){
            lim = 1.3f;
        }
        else {
            lim = 1.0f; //Bot back to normal speed
        }
    }

    void driveControl(){
        driveButtonCheck();

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //drive subsystem
        //Update the motor speeds
        driveSystem.update(gamepad1, heading, FIELD_CENTRIC, lim);
    }

    void armControl(){
        if(gamepad2.left_bumper) {
            armSystem.setClaw(0.75);
        } else if(gamepad2.right_bumper) {
            armSystem.setClaw(0);
        }

        //armSystem.setElbowPosition(-gamepad2.right_stick_y);
        armSystem.setRotation(gamepad2.right_stick_x);
        //armSystem.setShoulderPower(-gamepad2.left_stick_y);
        armSystem.setLiftPower(gamepad2.right_trigger - gamepad2.left_trigger);



        if(clawClosed.updateActivate(gamepad2.left_bumper)){
            armSystem.setClaw(clawClosedAngle);
        } else if(clawOpen.updateActivate(gamepad2.right_bumper)){
            armSystem.setClaw(clawOpenAngle);
        }

        if(gamepad2.b){
            armSystem.setRotation(rotationMid);
        }
    }

    void telemetryUpdate(){
        //husky.print();
        //Pose2d poseEstimate = driveSystem.getPoseEstimate(); //Ignore this, it doesn't work
        telemetry.addData("Mode is Field Centric", FIELD_CENTRIC);
        telemetry.addData("claw POS", armSystem.claw.getPosition());
        telemetry.addData("Ready for pixel", readyState);
        telemetry.addData("elbow ENC", armSystem.elbow.getPosition());
        telemetry.addData("shoulder ENC", armSystem.shoulder.getPosition());
        telemetry.addData("lift ENC", armSystem.lift.getCurrentPosition());
        //telemetry.addData("wrist POS", armSystem.wrist.getPosition());
        telemetry.update();
    }

}


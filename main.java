package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Main extends LinearOpMode {

    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor elbow;
    DcMotor slideRot;
    DcMotor lift;
    Servo clawRot;
    Servo grip;

    IMU imu;

    boolean isRobotCentric = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        slideRot = hardwareMap.dcMotor.get("slideRot");
        lift = hardwareMap.dcMotor.get("lift");
        clawRot = hardwareMap.servo.get("clawRot");
        grip = hardwareMap.servo.get("grip");
        elbow = hardwareMap.dcMotor.get("elbow");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB
        // forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            mechanumLoop();
            slideLoop();
            clawLoop();
        }
    }

    public void clawLoop(){
        if(gamepad2.dpad_up){//change to gamepad 2 - done
            clawRot.setPosition(90);
        }
        if(gamepad2.dpad_down){
            clawRot.setPosition(0);
        }
        if(gamepad2.dpad_left){
            grip.setPosition(180);
        }
        if(gamepad2.dpad_right){
            grip.setPosition(0);
        }
        elbow.setPower(gamepad2.left_stick_y);
    }

    public void mechanumLoop(){
        if(isRobotCentric){
            robotCentricLoop();
        }
        else{
            fieldCentricLoop();
        }
        if(gamepad1.a){
            isRobotCentric = !isRobotCentric;
            while(gamepad1.a){
                opModeIsActive();
            }
        }
    }

    public void slideLoop(){
        if(gamepad2.left_trigger != 0){///////////////change to gamepad 2 - done
            lift.setPower(gamepad1.left_trigger);
        }
        else if(gamepad2.right_trigger != 0){
            lift.setPower(gamepad1.right_trigger * (-1));
        }
        else{
            lift.setPower(0);
        }
        if(gamepad2.left_bumper){
            slideRot.setPower(-1);
        }
        else if(gamepad2.right_bumper){
            slideRot.setPower(1);
        }
        else{
            slideRot.setPower(0);
        }
    }

    public void fieldCentricLoop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if(gamepad1.b){
            frontLeftPower /= 3;
            backLeftPower /= 3;
            frontRightPower /= 3;
            backRightPower /= 3;
        }

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void robotCentricLoop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        
        if(gamepad1.b){
            frontLeftPower /= 3;
            backLeftPower /= 3;
            frontRightPower /= 3;
            backRightPower /= 3;
        }

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}

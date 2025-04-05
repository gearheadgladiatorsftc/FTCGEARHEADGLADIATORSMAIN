package org.firstinspires.ftc.teamcode;

//Import proper packages
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//We are creating a class called DriveSystem. It contains functions.
public class DriveSystem {
    // Declare variables. These variables are only accessible within this class
    SampleMecanumDrive drive;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    IMU imu;

    //This is the init function. Call this function to initialize the class, usually at
    //the beginning of the script
    public DriveSystem(HardwareMap hardwareMap) {
        //TODO: Figure out what SampleMecanumDrive does ;)
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //no encoder pls

        //set motor variables to the correct port
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        //Don't use encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init IMU (gyroscope)
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB
        // forward
        imu.initialize(parameters);
    }

    //This function updates the motor values.
    //Pass the function the gamepad to check values from, the current gyro heading
    //the drive mode, and the max motor speed
    //TODO: heading is not used, remove from functions and update function calls to not include ti
    public void update(Gamepad gamepad1, double heading, boolean FIELD_CENTRIC, float limit) {
        Vector2d input;

        if(FIELD_CENTRIC) { //If we are in Field centric mode...
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed, so multiply by -1
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x/2; //Don't question the algorithm, just trust that it works

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //

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

            //Divide motor values by the limit
            //don't limit motors to the limit, that will throw off the proportions
            frontLeftPower /= limit;
            backLeftPower /= limit;
            frontRightPower /= limit;
            backRightPower /= limit;

            //update motor speeds
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        } else { //If we are in robot-centric mode
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            //double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double x = ((gamepad1.right_trigger/2) - ((gamepad1.left_trigger)/2.0)) * 1.1;
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftPower /= limit;
            backLeftPower /= limit;
            frontRightPower /= limit;
            backRightPower /= limit;

            frontLeft.setPower(-frontLeftPower *0.9);
            backLeft.setPower(-backLeftPower *0.9);
            frontRight.setPower(-frontRightPower *0.9);
            backRight.setPower(-backRightPower *0.9);
        }
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
   }

}

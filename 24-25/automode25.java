package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "automode25", group = "Autonomous")
public class automode25 extends LinearOpMode {

    // Motors and servos
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor elbow;
    private Servo clawRot;
    private Servo grip;

    // Constants
    private static final double TICKS_PER_TILE = 2142; // Update based on your robot's specific calculation
    private static final double TICKS_PER_DEGREE = 10; // Example value; calculate based on turning tests
    private static final double TILE_LENGTH_INCHES = 24.0;

    @Override
    public void runOpMode() {
        // Hardware map setup
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        elbow = hardwareMap.dcMotor.get("elbow");
        clawRot = hardwareMap.servo.get("clawRot");
        grip = hardwareMap.servo.get("grip");

        // Reverse the right-side motors
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        resetEncoders();

        waitForStart();

        if (opModeIsActive()) {
            // Autonomous sequence
            moveForward(1.5);  // Move forward 1.5 tiles
            turnRight(90);     // Turn 90˚ to the right
            moveForward(1.5);  // Move forward 1.5 tiles
            turnLeft(90);      // Turn 90˚ to the left
            grabSample();      // Grab sample using servo claw
            turnRight(180);    // Turn 180˚
            moveForward(1);    // Move forward 1 tile
            dropSample();      // Drop sample using servo claw
            turnRight(180);    // Turn 180˚
            moveForward(1);    // Move forward 1 tile
            strafeRight(0.5);  // Strafe 0.5 tiles to align
            grabSample();      // Grab another sample
            moveForward(0.5);  // Move forward 0.5 tiles
            turnRight(90);     // Turn 90˚ to the right
            moveForward(1);    // Move forward 1 tile
            dropSample();      // Drop the sample
            turnRight(180);    // Turn 180˚
            moveForward(1);    // Move forward 1 tile
            strafeLeft(0.5);   // Strafe 0.5 tiles to align
            grabSample();      // Grab another sample
            moveForward(1);    // Move forward 1 tile
            dropSample();      // Drop the sample
        }
    }

    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveForward(double tiles) {
        int targetTicks = (int) (tiles * TICKS_PER_TILE);

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + targetTicks);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + targetTicks);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + targetTicks);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + targetTicks);

        setRunToPosition();

        setMotorPower(0.5);

        while (opModeIsActive() && motorsBusy()) {
            idle();
        }

        setMotorPower(0);
    }

    private void turnRight(int degrees) {
        int targetTicks = (int) (degrees * TICKS_PER_DEGREE);

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + targetTicks);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + targetTicks);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - targetTicks);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() - targetTicks);

        setRunToPosition();

        setMotorPower(0.5);

        while (opModeIsActive() && motorsBusy()) {
            idle();
        }

        setMotorPower(0);
    }

    private void turnLeft(int degrees) {
        turnRight(-degrees);
    }

    private void strafeRight(double tiles) {
        int targetTicks = (int) (tiles * TICKS_PER_TILE);

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + targetTicks);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - targetTicks);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - targetTicks);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + targetTicks);

        setRunToPosition();

        setMotorPower(0.5);

        while (opModeIsActive() && motorsBusy()) {
            idle();
        }

        setMotorPower(0);
    }

    private void strafeLeft(double tiles) {
        strafeRight(-tiles);
    }

    private void grabSample() {
        grip.setPosition(0); // Adjust grip to close
    }

    private void dropSample() {
        grip.setPosition(1); // Adjust grip to release
    }

    private void setRunToPosition() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setMotorPower(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    private boolean motorsBusy() {
        return frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy();
    }
}

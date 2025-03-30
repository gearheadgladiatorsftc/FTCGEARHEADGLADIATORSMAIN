package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GpioPin;
import com.qualcomm.robotcore.hardware.GpioPinDigitalInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Center-Back", group = "")
public class Main extends LinearOpMode {

  //Change these variables depending on the starting position.
  final double startingPosX = 0;
  final double startingPosY = 0;

  double x = startingPosX;
  double y = startingPosY;

  private static DcMotor frontRightMotor;
  private static DcMotor backRightMotor;
  private static DcMotor frontLeftMotor;
  private static DcMotor backLeftMotor;

  @Override
  public void runOpMode() {
    waitForStart();

    while (opModeIsActive()) {
      // Autonomous logic here

      // This is where you would put code to control your robot during autonomous mode
    }
  }

  public static void logPosition() {}

  public static void checkForObstacles() {}

  public static void scanLidar() {}

  public class RobotController {

    // Method to navigate the robot to a specific position (x, y)
    public void moveToPosition(float x, float y, float currentX, float currentY) {
        // Calculate the angle the robot needs to turn to face the target position
        float angleToTurn = calculateAngle(currentX, currentY, x, y);

        // Turn the robot to face the target position
        turn(angleToTurn);

        // Calculate the distance to move to reach the target position
        float distanceToMove = calculateDistance(currentX, currentY, x, y);

        // Check if the distance should be negative
        boolean direction = true; // By default, move forward
        if (distanceToMove < 0) {
            distanceToMove = -distanceToMove; // Make distance positive
            direction = false; // Set direction to move backward
        }

        // Stop execution if distance to move is 0
        if (distanceToMove == 0) {
            return;
        }

        // Move the robot forward or backward by the calculated distance
        move(direction, distanceToMove);
    }

    // Method to calculate the angle the robot needs to turn to face the target position
    private float calculateAngle(float currentX, float currentY, float targetX, float targetY) {
        float deltaX = targetX - currentX;
        float deltaY = targetY - currentY;

        // Calculate the angle using arctangent
        float angleInRadians = (float) Math.atan2(deltaY, deltaX);
        float angleInDegrees = (float) Math.toDegrees(angleInRadians);

        // Ensure the angle is between 0 and 360 degrees
        if (angleInDegrees < 0) {
            angleInDegrees += 360;
        }

        return angleInDegrees;
    }

    // Method to calculate the distance between two points
    private float calculateDistance(float x1, float y1, float x2, float y2) {
        float deltaX = x2 - x1;
        float deltaY = y2 - y1;

        // Calculate the Euclidean distance
        return (float) Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    // Method to turn the robot to a specific angle
    private void turn(float degrees) {
        // Implementation of turn() method
    }

    // Method to move the robot forward or backward by a specified distance
    private void move(boolean direction, float inches) {
        // Implementation of move() method
    }
}

}

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

  public static void moveToPosition(double targetx, double targety) {}
}

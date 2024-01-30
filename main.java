package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.GpioPin;
import com.qualcomm.robotcore.hardware.GpioPinDigitalInput;

@TeleOp(name = "Competition", group = "Competition")
public class mecanumdrivesample1 extends LinearOpMode {//#########################################################################################################

  private DcMotor frontRightMotor;
  private DcMotor backRightMotor;
  private DcMotor frontLeftMotor;
  private DcMotor backLeftMotor;
  private HardwareMap hwMap = hardwareMap;
  private GpioPin sIn = hwMap.get(GpioPin.class, "slideIn");
  private GpioPin sOut = hwMap.get(GpioPin.class, "slideOut");

  /**
   * This function is executed when this Op Mode is selected from the Driver
   * Station.
   */
  @Override
  public void runOpMode() {/*************************************************************************************************************************************/
    // mechanum variables
    float y;
    double x;
    float rx;
    double denominator;

    boolean pixelLoaded = false;
    boolean grabbed = false; //is the pixel grabbed?

    // threshold of low battery variable
    float lowBatThreshold = 9.0;

    boolean autobreak = true; //check if slide autobreak is on
    boolean slideDown = true; //Check if the slide is down

    frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");

    // Reverse the right side motors. This may be wrong for your setup.
    // If your robot moves backwards when commanded to go forwards, reverse the left
    // side instead.
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    // private HuskyLens huskylensAsHuskyLens;
    ElapsedTime myElapsedTime;
    List<HuskyLens.Block> myHuskyLensBlocks;
    // HuskyLens.Block myHuskyLensBlock;

    // huskylensAsHuskyLens = hardwareMap.get(HuskyLens.class,
    // "huskylensAsHuskyLens");
    float batvolt = ControlHub_VoltageSensor.getVoltage();
    telemetry.addLine("Current Battery Voltage (v): " + batvolt);
    // telemetry.update();
    if (batvolt <= lowBatThreshold) {
      telemetry.addLine("Voltage too low. Please charge or replace battery.");
      telemetry.update();
    }
    // Put initialization blocks here.
    //telemetry.addData(">>",
    //huskylensAsHuskyLens.knock() ? "Touch start to continue" : "Problem communicating with HuskyLens");
    // huskylensAsHuskyLens.selectAlgorithm();
    //telemetry.update();
    myElapsedTime = new ElapsedTime();

    sOut.setState(true);

    waitForStart();
    while (opModeIsActive()) {
      updateGPIO();
      mechanumLoop(); // Run the loop for the mechanum drive
      huskyLoop(); // Run the loop to check for a new HuskyLens inference
      slideLoop();// Run the loop to control the slide
    }

  }

   static void huskyLoop() {/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
     * if (myElapsedTime.seconds() >= 1) {
     * myElapsedTime.reset();
     * myHuskyLensBlocks = huskylensAsHuskyLens.blocks();
     * telemetry.addData("Block count", JavaUtil.listLength(myHuskyLensBlocks));
     * for (HuskyLens.Block myHuskyLensBlock_item : myHuskyLensBlocks) {
     * myHuskyLensBlock = myHuskyLensBlock_item;
     * telemetry.addData("Block", "id=" + myHuskyLensBlock.id + " size: " +
     * myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " +
     * myHuskyLensBlock.x + "," + myHuskyLensBlock.y;
     * }
     * telemetry.update();
     * }
     */
  }

  public void mechanumLoop() {/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Remember, Y stick value is reversed
    y = -gamepad1.left_stick_y;
    // Factor to counteract imperfect strafing
    x = gamepad1.left_stick_x * 1.1;
    rx = gamepad1.right_stick_x;
    // Denominator is the largest motor power (absolute value) or 1.
    // This ensures all powers maintain the same ratio, but only if one is outside
    // of the range [-1, 1].
    denominator = JavaUtil.maxOfList(JavaUtil
        .createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
    // Make sure your ID's match your configuration
    frontLeftMotor.setPower((y + x + rx) / denominator);
    backLeftMotor.setPower(((y - x) + rx) / denominator);
    frontRightMotor.setPower(((y - x) - rx) / denominator);
    backRightMotor.setPower(((y + x) - rx) / denominator);
  }
  public boolean checkIfDown(){ //Method that checks to see if the slide is resting against the robot
    //Code to use distance sensor
    return true;
  }
  public void slideLoop(){/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(gamepad2.b && !autobreak){
      //brake motor
      return;//terminate the rest of the loop
    }
    else{
        //unbreak motor
    }
    if (gamepad1.left_trigger > 0 && !slideDown()) {// Check if the trigger is depressed and the slide is up
      lift.setPower(gamepad2.left_trigger); //move slide down
    } else if (gamepad1.right_trigger > 0) { //check if trigger is depressed
      lift.setPower(-1 * gamepad2.right_trigger); // move slide up
    } else {
      lift.setPower(0);
      if(autobreak){
        //autobreak
      }
    }
    if (gamepad2.x && !autobreakPressed) {
      autobreak = !autobreak;
      autobreakPressed = true;
      autobreakPressedTimestamp = System.currentTimeMillis();
  }
  
  // Add these member variables at the beginning of your class
  private boolean autobreakPressed = false;
  private long autobreakPressedTimestamp = 0;
  
  // Add this method in your class
  private void resetAutobreakButton() {
      autobreakPressed = false;
      autobreakPressedTimestamp = 0;
  }
  
  // Then, in your main loop or a more suitable place, add the following code:
  if (System.currentTimeMillis() - autobreakPressedTimestamp > 1000) {
      resetAutobreakButton();
  }
  
  // Also, call resetAutobreakButton() when the op mode is starting, or when needed to reset the button state.
  
  }
  public void clawLoop(){//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (gamepad2.y && pixelLoaded && !checkIfDown() && !liftMovingDown) {
      lift.setPower(-1);
      liftMovingDown = true;
      liftMovingDownTimestamp = System.currentTimeMillis();
  }
  
  // Add these member variables at the beginning of your class
  private boolean liftMovingDown = false;
  private long liftMovingDownTimestamp = 0;
  
  // Add this method in your class
  private void resetLiftMovingDown() {
      liftMovingDown = false;
      liftMovingDownTimestamp = 0;
  }
  
  // Then, in your main loop or a more suitable place, add the following code:
  if (System.currentTimeMillis() - liftMovingDownTimestamp > 1000) {
      resetLiftMovingDown();
  }
  
  // Also, call resetLiftMovingDown() when the op mode is starting, or when needed to reset the button state.
  
    if(gamepad2.a){
        if(grabbed){
            //release pixel
            grabbed = false;
        }
        else{
            //grab pixel
            grabbed = true;
        }
        while(gamepad2.a){}
    }
  }
  public void updateGPIO(){
    slideDown = ((GpioPinDigitalInput) sIn).getState();
  }
}

/*
 * Notes for next time:
 * 
 * left joystick moves claw angle
 */

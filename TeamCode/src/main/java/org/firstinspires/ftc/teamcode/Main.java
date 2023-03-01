package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main")
public class Main extends LinearOpMode {

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
    DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
    Servo left = hardwareMap.get(Servo.class, "left");
    Servo right = hardwareMap.get(Servo.class, "right");
    DcMotor arm1 = hardwareMap.get(DcMotor.class, "arm1");
    DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");

    // Reverse the right side.
    rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
    leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
    leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    arm1.setDirection(DcMotorSimple.Direction.REVERSE);
    // This makes the robot BRAKE when power becomes zero. The other
    // mode, FLOAT, makes the robot go in neutral and will drift.
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        telemetry.addLine(String.format("\nArm 1 Position: %d", arm1.getCurrentPosition()));
        telemetry.addLine(String.format("\nArm 2 Position: %d", arm2.getCurrentPosition()));
        if (gamepad2.x) {
          left.setPosition(0);
          right.setPosition(0.4);
        } else if (gamepad2.y) {
          left.setPosition(0.4);
          right.setPosition(0);
        }
        if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x > 0.5) {
          rightFront.setPower(-0.75);
          leftFront.setPower(0);
          rightRear.setPower(0);
          leftRear.setPower(-0.75);
        } else if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x < -0.5) {
          rightFront.setPower(0);
          leftFront.setPower(-0.75);
          rightRear.setPower(-0.75);
          leftRear.setPower(0);
        } else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x < -0.5) {
          rightFront.setPower(0.75);
          leftFront.setPower(0);
          rightRear.setPower(0);
          leftRear.setPower(0.75);
        } else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x > 0.5) {
          rightFront.setPower(0);
          leftFront.setPower(0.75);
          rightRear.setPower(0.75);
          leftRear.setPower(0);
        } else if (gamepad1.left_stick_y > 0.5) {
          rightFront.setPower(-0.75);
          leftFront.setPower(-0.75);
          rightRear.setPower(-0.75);
          leftRear.setPower(-0.75);
        } else if (gamepad1.left_stick_y < -0.5) {
          rightFront.setPower(0.75);
          leftFront.setPower(0.75);
          rightRear.setPower(0.75);
          leftRear.setPower(0.75);
        } else if (gamepad1.left_stick_x > 0.5) {
          rightFront.setPower(-0.75);
          leftFront.setPower(0.75);
          rightRear.setPower(0.75);
          leftRear.setPower(-0.75);
        } else if (gamepad1.left_stick_x < -0.5) {
          rightFront.setPower(0.75);
          leftFront.setPower(-0.75);
          rightRear.setPower(-0.75);
          leftRear.setPower(0.75);
        } else if (gamepad1.right_bumper) {
          rightFront.setPower(0.75);
          leftFront.setPower(-0.75);
          rightRear.setPower(0.75);
          leftRear.setPower(-0.75);
        } else if (gamepad1.left_bumper) {
          rightFront.setPower(-0.75);
          leftFront.setPower(0.75);
          rightRear.setPower(-0.75);
          leftRear.setPower(0.75);
        } else {
          rightFront.setPower(0);
          leftFront.setPower(0);
          rightRear.setPower(0);
          leftRear.setPower(0);
        }
        if (gamepad1.right_stick_x > 0.5) {
          rightFront.setPower(0.75);
          leftFront.setPower(-0.75);
          rightRear.setPower(0.75);
          leftRear.setPower(-0.75);
        } else if (gamepad1.right_stick_x < -0.5) {
          rightFront.setPower(-0.75);
          leftFront.setPower(0.75);
          rightRear.setPower(-0.75);
          leftRear.setPower(0.75);
        }
        if (gamepad2.right_trigger > 0.1) {
          arm1.setPower(gamepad2.right_trigger);
          arm2.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1) {
          arm1.setPower(-1 * gamepad2.left_trigger);
          arm2.setPower(-1 * gamepad2.left_trigger);
        } else {
          arm1.setPower(0);
          arm2.setPower(0);
        }
      }
    }
  }
}

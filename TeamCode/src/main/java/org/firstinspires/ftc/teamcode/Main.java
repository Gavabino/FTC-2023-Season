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
    DcMotor right_front = hardwareMap.get(DcMotor.class, "right front");
    DcMotor left_front = hardwareMap.get(DcMotor.class, "left front");
    DcMotor right_rear = hardwareMap.get(DcMotor.class, "right rear");
    DcMotor left_rear = hardwareMap.get(DcMotor.class, "left rear");
    Servo left = hardwareMap.get(Servo.class, "left");
    Servo right = hardwareMap.get(Servo.class, "right");
    DcMotor arm1 = hardwareMap.get(DcMotor.class, "arm1");
    DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");

    // Reverse the right side.
    right_front.setDirection(DcMotorSimple.Direction.FORWARD);
    left_front.setDirection(DcMotorSimple.Direction.REVERSE);
    right_rear.setDirection(DcMotorSimple.Direction.REVERSE);
    left_rear.setDirection(DcMotorSimple.Direction.REVERSE);
    // This makes the robot BRAKE when power becomes zero. The other
    // mode, FLOAT, makes the robot go in neutral and will drift.
    right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        if (gamepad2.x) {
          left.setPosition(0);
          right.setPosition(0.4);
        } else if (gamepad2.y) {
          left.setPosition(0.4);
          right.setPosition(0);
        }
        if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x > 0.5) {
          right_front.setPower(0);
          left_front.setPower(0.75);
          right_rear.setPower(0.75);
          left_rear.setPower(0);
        } else if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x < -0.5) {
          right_front.setPower(0.75);
          left_front.setPower(0);
          right_rear.setPower(0);
          left_rear.setPower(0.75);
        } else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x < -0.5) {
          right_front.setPower(0);
          left_front.setPower(-0.75);
          right_rear.setPower(-0.75);
          left_rear.setPower(0);
        } else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x > 0.5) {
          right_front.setPower(-0.75);
          left_front.setPower(0);
          right_rear.setPower(0);
          left_rear.setPower(-0.75);
        } else if (gamepad1.left_stick_y > 0.5) {
          right_front.setPower(-0.75);
          left_front.setPower(-0.75);
          right_rear.setPower(-0.75);
          left_rear.setPower(-0.75);
        } else if (gamepad1.left_stick_y < -0.5) {
          right_front.setPower(0.75);
          left_front.setPower(0.75);
          right_rear.setPower(0.75);
          left_rear.setPower(0.75);
        } else if (gamepad1.left_stick_x > 0.5) {
          right_front.setPower(-0.75);
          left_front.setPower(0.75);
          right_rear.setPower(0.75);
          left_rear.setPower(-0.75);
        } else if (gamepad1.left_stick_x < -0.5) {
          right_front.setPower(0.75);
          left_front.setPower(-0.75);
          right_rear.setPower(-0.75);
          left_rear.setPower(0.75);
        } else if (gamepad1.right_bumper) {
          right_front.setPower(0.75);
          left_front.setPower(-0.75);
          right_rear.setPower(0.75);
          left_rear.setPower(-0.75);
        } else if (gamepad1.left_bumper) {
          right_front.setPower(-0.75);
          left_front.setPower(0.75);
          right_rear.setPower(-0.75);
          left_rear.setPower(0.75);
        } else {
          right_front.setPower(0);
          left_front.setPower(0);
          right_rear.setPower(0);
          left_rear.setPower(0);
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

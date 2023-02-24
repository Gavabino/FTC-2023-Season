package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(name = "Main2")
public class Main2 extends LinearOpMode {

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Servo left = hardwareMap.get(Servo.class, "left");
    Servo right = hardwareMap.get(Servo.class, "right");
    DcMotor arm1 = hardwareMap.get(DcMotor.class, "arm1");
    DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");
    arm1.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        telemetry.addLine(String.format("\nArm 1 Position: %d", arm1.getCurrentPosition()));
        telemetry.addLine(String.format("\nArm 2 Position: %d", arm2.getCurrentPosition()));
        telemetry.update();
        telemetry.clear();
        if (gamepad2.x) {
          left.setPosition(0);
          right.setPosition(0.4);
        } else if (gamepad2.y) {
          left.setPosition(0.4);
          right.setPosition(0);
        }
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                         gamepad1.right_stick_x
                )
        );

        drive.update();
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

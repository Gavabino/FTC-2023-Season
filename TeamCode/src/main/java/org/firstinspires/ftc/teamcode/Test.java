package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor arm1 = hardwareMap.get(DcMotor.class, "arm1");
        DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    arm1.setTargetPosition(0);
                    arm2.setTargetPosition(0);
                    arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (arm1.getCurrentPosition() < 10 && opModeIsActive()) {
                        if (arm1.getCurrentPosition() > 0 && arm2.getCurrentPosition() < 0) {
                            arm1.setPower(1);
                            arm2.setPower(1);
                        } else if (arm1.getCurrentPosition() < 0 || arm2.getCurrentPosition() > 0) {
                            arm1.setPower(0);
                            arm2.setPower(0);
                        } else {
                            arm1.setPower(-1);
                            arm2.setPower(-1);
                        }
                    }
                }
                if (gamepad1.b) {
                    arm1.setTargetPosition(1500);
                    arm2.setTargetPosition(-1500);
                    arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (arm1.getCurrentPosition() <= 1500 && opModeIsActive()) {
                        if (arm1.getCurrentPosition() < 1500 && arm2.getCurrentPosition() > -1500) {
                            arm1.setPower(-1);
                            arm2.setPower(-1);
                        } else if (arm1.getCurrentPosition() > 1500 || arm2.getCurrentPosition() < -1500) {
                            arm1.setPower(0);
                            arm2.setPower(0);
                        } else {
                            arm1.setPower(1);
                            arm2.setPower(1);
                        }

                    }
                }
                telemetry.addData("Arm 1 Position: ", arm1.getCurrentPosition());
                telemetry.addData("Arm 2 Position: ", arm2.getCurrentPosition());
                telemetry.addData("Arm 1 Power: ", arm1.getPower());
                telemetry.addData("Arm 2 Power: ", arm2.getPower());
                telemetry.update();
                telemetry.clear();
            }
        }
    }
}
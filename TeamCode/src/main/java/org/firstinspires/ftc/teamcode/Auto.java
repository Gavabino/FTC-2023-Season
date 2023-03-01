package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Config
public class Auto extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tag_size = 0.166;

    // Tag ID 1,2,3 from the 36h11 family

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest;

    @Override
    public void runOpMode()
    {
        @SuppressLint("DiscouragedApi") final int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(this.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.aprilTagDetectionPipeline = new AprilTagDetectionPipeline(this.tag_size, this.fx, this.fy, this.cx, this.cy);

        this.camera.setPipeline(this.aprilTagDetectionPipeline);
        this.camera.openCameraDeviceAsync(new AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Auto.this.camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(final int errorCode)
            {

            }
        });

        this.telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.
        final SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);

        final Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(2.8)
                .build();
        final Trajectory middle = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();
        final Trajectory left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(3.2)
                .build();
        final Trajectory right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(3.2)
                .build();
        final Trajectory to_pole1 = drive.trajectoryBuilder(new Pose2d())
                .forward(1.5)
                .build();
        final Trajectory to_pole2 = drive.trajectoryBuilder(to_pole1.end())
                .strafeRight(0.6)
                .build();
        final Trajectory start = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(0.2)
                .build();

        Servo left_servo = hardwareMap.get(Servo.class, "left");
        Servo right_servo = hardwareMap.get(Servo.class, "right");
        DcMotor arm1 = hardwareMap.get(DcMotor.class, "arm1");
        DcMotor arm2 = hardwareMap.get(DcMotor.class, "arm2");
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!this.isStarted() && !this.isStopRequested())
        {
            final ArrayList<AprilTagDetection> currentDetections = this.aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(final AprilTagDetection tag : currentDetections)
                    if (tag.id == this.LEFT || tag.id == this.MIDDLE || tag.id == this.RIGHT) {
                        this.tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }

                if(tagFound)
                {
                    this.telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    this.tagToTelemetry(this.tagOfInterest);
                }
                else
                {
                    this.telemetry.addLine("Don't see tag of interest :(");

                    if(this.tagOfInterest == null)
                        this.telemetry.addLine("(The tag has never been seen)");
                    else
                    {
                        this.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        this.tagToTelemetry(this.tagOfInterest);
                    }
                }

            }
            else
            {
                this.telemetry.addLine("Don't see tag of interest :(");

                if(this.tagOfInterest == null)
                    this.telemetry.addLine("(The tag has never been seen)");
                else
                {
                    this.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    this.tagToTelemetry(this.tagOfInterest);
                }

            }

            this.telemetry.update();
            this.sleep(20);
        }





        if(this.tagOfInterest != null)
        {
            this.telemetry.addLine("Tag snapshot:\n");
            this.tagToTelemetry(this.tagOfInterest);
            this.telemetry.update();
        }
        else
        {
            this.telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            this.telemetry.update();
        }

        //PUT AUTO CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        left_servo.setPosition(0);
        right_servo.setPosition(0.4);
        sleep(200);

        arm1.setPower(1);
        arm2.setPower(1);
        sleep(200);
        arm1.setPower(0);
        arm2.setPower(0);
        sleep(100);

        drive.followTrajectory(start);
        sleep(100);

        drive.followTrajectory(to_pole1);
        sleep(100);

        drive.followTrajectory(to_pole2);
        //default path
        if(this.tagOfInterest == null) {

        }else switch (this.tagOfInterest.id) {
            case 1:
                drive.followTrajectory(forward);
                sleep(100);
                drive.followTrajectory(left);
                break;
            case 2:
                drive.followTrajectory(middle);
                break;
            case 3:
                drive.followTrajectory(forward);
                sleep(100);
                drive.followTrajectory(right);
                break;
        }



    }

    void tagToTelemetry(final AprilTagDetection detection)
    {
        this.telemetry.addData("Detected tag ID: ", detection.id);
        this.telemetry.addData("Translation X: ", detection.pose.x* Auto.FEET_PER_METER);
        this.telemetry.addData("Translation Y: ", detection.pose.y* Auto.FEET_PER_METER);
        this.telemetry.addData("Translation Z: ", detection.pose.z* Auto.FEET_PER_METER);
        this.telemetry.addData("Rotation Yaw: ", Math.toDegrees(detection.pose.yaw));
        this.telemetry.addData("Rotation Pitch: ", Math.toDegrees(detection.pose.pitch));
        this.telemetry.addData("Rotation Roll: ", Math.toDegrees(detection.pose.roll));
    }
}

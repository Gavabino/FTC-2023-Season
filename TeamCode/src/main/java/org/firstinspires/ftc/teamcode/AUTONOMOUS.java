/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous
public class AUTONOMOUS extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
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
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    Trajectory Left1 = this.drive.trajectoryBuilder(new Pose2d())
            .forward(3)
            .build();
    Trajectory Left2 = this.drive.trajectoryBuilder(this.Left1.end())
            .strafeLeft(3)
            .build();
    Trajectory Left3 = this.drive.trajectoryBuilder(this.Left2.end())
            .forward(3)
            .build();

    Trajectory Middle = this.drive.trajectoryBuilder(new Pose2d())
            .forward(5)
            .build();
    Trajectory Right1 = this.drive.trajectoryBuilder(new Pose2d())
            .forward(3)
            .build();
    Trajectory Right2 = this.drive.trajectoryBuilder(this.Right1.end())
            .strafeLeft(3)
            .build();
    Trajectory Right3 = this.drive.trajectoryBuilder(this.Right2.end())
            .forward(3)
            .build();
    AprilTagDetection tagOfInterest = null;
    private DcMotor arm1 = null;
    private DcMotor arm2 = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(this.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.aprilTagDetectionPipeline = new AprilTagDetectionPipeline(this.tagsize, this.fx, this.fy, this.cx, this.cy);

        this.camera.setPipeline(this.aprilTagDetectionPipeline);
        this.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                AUTONOMOUS.this.camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        this.telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.
        this.arm1 = this.hardwareMap.get(DcMotor.class, "arm1");
        this.arm2 = this.hardwareMap.get(DcMotor.class, "arm2");
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!this.isStarted() && !this.isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = this.aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.isEmpty()) {
                this.telemetry.addLine("Don't see tag of interest :(");

                if (null == this.tagOfInterest) {
                    this.telemetry.addLine("(The tag has never been seen)");
                } else {
                    this.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    this.tagToTelemetry(this.tagOfInterest);
                }

            } else {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == this.LEFT || tag.id == this.MIDDLE || tag.id == this.RIGHT) {
                        this.tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    this.telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    this.tagToTelemetry(this.tagOfInterest);
                } else {
                    this.telemetry.addLine("Don't see tag of interest :(");

                    if (this.tagOfInterest == null) {
                        this.telemetry.addLine("(The tag has never been seen)");
                    } else {
                        this.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        this.tagToTelemetry(this.tagOfInterest);
                    }
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

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)


        if(this.tagOfInterest == null){
            //default path
            this.drive.followTrajectory(this.Middle);
            this.telemetry.addData("Final Heading: ", this.drive.getPoseEstimate());
            this.telemetry.addData("Arm1 Position: ", this.arm1.getCurrentPosition());
            this.telemetry.addData("Arm2 Position", this.arm2.getCurrentPosition());
        }else{
            switch(this.tagOfInterest.id){
                case 1:
                    this.drive.followTrajectory(this.Left1);
                    this.drive.followTrajectory(this.Left2);
                    this.drive.followTrajectory(this.Left3);
                    this.telemetry.addData("Final heading: ", this.drive.getPoseEstimate());
                    this.telemetry.addData("Arm1 Position: ", this.arm1.getCurrentPosition());
                    this.telemetry.addData("Arm2 Position", this.arm2.getCurrentPosition());
                    break;
                case 2:
                    this.drive.followTrajectory(this.Middle);
                    this.telemetry.addData("Final heading: ", this.drive.getPoseEstimate());
                    this.telemetry.addData("Arm1 Position: ", this.arm1.getCurrentPosition());
                    this.telemetry.addData("Arm2 Position", this.arm2.getCurrentPosition());
                    break;
                case 3:
                    this.drive.followTrajectory(this.Right1);
                    this.drive.followTrajectory(this.Right2);
                    this.drive.followTrajectory(this.Right3);
                    this.telemetry.addData("Final heading: ", this.drive.getPoseEstimate());
                    this.telemetry.addData("Arm1 Position: ", this.arm1.getCurrentPosition());
                    this.telemetry.addData("Arm2 Position", this.arm2.getCurrentPosition());
                    break;
            }
        }



    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        this.telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
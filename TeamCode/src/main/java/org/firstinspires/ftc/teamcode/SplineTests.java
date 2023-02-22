package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTests extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-36.11, -61.55, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-38.16, -41.11), Math.toRadians(70.94))
                .splineTo(new Vector2d(0.23, -27.26), Math.toRadians(90.01))
                .splineTo(new Vector2d(-0.68, -38.84), Math.toRadians(265.52))
                .splineTo(new Vector2d(-14.99, -41.79), Math.toRadians(191.66))
                .splineTo(new Vector2d(-35.66, -36.11), Math.toRadians(164.64))
                .build();

        drive.followTrajectorySequence(traj);
    }
}

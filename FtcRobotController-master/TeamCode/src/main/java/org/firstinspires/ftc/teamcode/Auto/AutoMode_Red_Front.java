package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="SimpleTest")
public class AutoMode_Red_Front extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-71, -45, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d()).strafeRight(4).build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d()).forward(50).build();



        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
    }
}

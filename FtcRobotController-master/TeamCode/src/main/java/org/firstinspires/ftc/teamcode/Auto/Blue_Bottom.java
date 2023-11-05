package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//102 in
@Autonomous(name="Bottom_Red")
public class Blue_Bottom extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();


        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(3)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(45)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(10)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();



        drive.followTrajectory(traj);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

    }
}

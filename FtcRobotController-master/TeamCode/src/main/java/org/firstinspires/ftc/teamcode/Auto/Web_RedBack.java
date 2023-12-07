package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Web_RedBack")
public class Web_RedBack extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_main.tflite";

    private static final String[] LABELS = {
            "Prop",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() {

        initTfod();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            long startTime = System.nanoTime();
            while (opModeIsActive()) {
                telemetryTfod(startTime);
                telemetry.update();
                sleep(20);
            }
        }

        visionPortal.close();

    }

    private void initTfod() {

        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.55f);

    }

    private void telemetryTfod(long startTime) {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        //telemetry.addData("# Objects Detected", currentRecognitions.size());
        int bond = 220;
        long currentTime = System.nanoTime();
        long time = (currentTime - startTime) / 1000000;

        if (time > 9000){
            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            telemetry.addLine("Right Spike");
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeLeft(5)
                    .forward(25)
                    .turn(Math.toRadians(-90))
                    .forward(15)
                    .back(13)
                    .turn(Math.toRadians(90))
                    .forward(23)
                    .turn(Math.toRadians(-90))
                    .forward(100)
                            .build();
            drive.followTrajectorySequence(trajSeq);
            requestOpModeStop();
        }

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            telemetry.addData("X", x);
            telemetry.update();
            if(x > bond){
                telemetry.addLine("Center");
                telemetry.update();

                Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(32)
                        .back(15)
                        .strafeLeft(15)
                        .forward(30)
                        .turn(Math.toRadians(-90))
                        .forward(105)
                                .build();

                waitForStart();

                if (isStopRequested()) return;

                drive.followTrajectorySequence(trajSeq);
                Pose2d poseEstimate = drive.getPoseEstimate();
                requestOpModeStop();


            } else{
                telemetry.addLine("Left spike");
                telemetry.update();
                Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(23)
                        .strafeLeft(9)
                        //.forward(11)
                        .back(11)
                        .strafeRight(13)
                        .forward(40)
                        .turn(Math.toRadians(-90))
                        .forward(90)
                        .build();

                drive.followTrajectorySequence(trajSeq);
                requestOpModeStop();
                //Pose2d poseEstimate = drive.getPoseEstimate();
            }
        }

    }

}

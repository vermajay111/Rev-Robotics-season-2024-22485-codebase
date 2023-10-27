package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Mecnaum")
public class RobotCentricMecnaum extends LinearOpMode {
    private DistanceSensor distanceSensorFront;
    //private DistanceSensor distanceSensorBack;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        YawPitchRollAngles RobotOrientation;
        distanceSensorFront = hardwareMap.get(DistanceSensor.class, "DistanceSensorFront");
        //distanceSensorBack = hardwareMap.get(DistanceSensor.class, "DistanceSensorBack");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        RobotOrientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        double max_speed = 1.0;
        boolean safe_mode = true;
        while (opModeIsActive()) {
            RobotOrientation = imu.getRobotYawPitchRollAngles();

            if(gamepad1.dpad_left && max_speed >= 0){
                max_speed -= 0.1;
            } else if(gamepad1.dpad_right && max_speed <= 1.0 ){
                max_speed += 0.1;
            }


            /*
            if (gamepad1.back){
                if(RobotOrientation.getYaw(AngleUnit.DEGREES) > 0){
                    while(RobotOrientation.getYaw(AngleUnit.DEGREES) > 0) {
                        frontLeftMotor.setPower(0.4);
                        backLeftMotor.setPower(0.4);
                        frontRightMotor.setPower(-0.4);
                        backRightMotor.setPower(-0.4);
                        RobotOrientation = imu.getRobotYawPitchRollAngles();
                        telemetry.update();
                    }
                } else if(RobotOrientation.getYaw(AngleUnit.DEGREES) < 0) {
                    while (RobotOrientation.getYaw(AngleUnit.DEGREES) < 0) {
                        frontLeftMotor.setPower(-0.4);
                        backLeftMotor.setPower(-0.4);
                        frontRightMotor.setPower(0.4);
                        backRightMotor.setPower(0.4);
                        RobotOrientation = imu.getRobotYawPitchRollAngles();
                        telemetry.update();
                    }
                }
            }

            if(gamepad1.y){
                safe_mode = !safe_mode;
            }

            */


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.01;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator) * max_speed;
            double backLeftPower = ((y - x + rx) / denominator) * max_speed;
            double frontRightPower = ((y - x - rx) / denominator) * max_speed;
            double backRightPower = ((y + x - rx) / denominator) * max_speed;


            /*
            if(safe_mode) {
                if ((distanceSensorFront.getDistance(DistanceUnit.CM) - 9.5) < 10 && frontLeftPower > 0.5 && backLeftPower > 0.5 && frontRightPower > 0.5 && backRightPower > 0.5){
                    frontLeftPower = 0;
                    backLeftPower = 0;
                    frontRightPower = 0;
                    backRightPower = 0;

                }
                else if ((distanceSensorFront.getDistance(DistanceUnit.CM) - 9.5) < 120 && frontLeftPower > 0.1 && backLeftPower > 0.1 && frontRightPower > 0.1 && backRightPower > 0.1) {
                    frontLeftPower *= 0.3;
                    backLeftPower *= 0.3;
                    frontRightPower *= 0.3;
                    backRightPower *= 0.3;
                }
                /*
                if ((distanceSensorFront.getDistance(DistanceUnit.CM)) < 7 && frontLeftPower < -0.5 && backLeftPower < -0.5 && frontRightPower < -0.5 && backRightPower < -0.5){
                    frontLeftPower = 0;
                    backLeftPower = 0;
                    frontRightPower = 0;
                    backRightPower = 0;
                }
                else if ((distanceSensorFront.getDistance(DistanceUnit.CM)) < 120 && frontLeftPower < -0.5 && backLeftPower < -0.5 && frontRightPower < -0.5 && backRightPower < -0.5){
                    frontLeftPower *= 0.5;
                    backLeftPower *= 0.5;
                    frontRightPower *= 0.5;
                    backRightPower *= 0.5;
                }

            }

            */


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //telemetry.addData("Main X degrees", RobotOrientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Front Sensor Distance", distanceSensorFront.getDistance(DistanceUnit.CM));
            telemetry.addData("Front Left Motor", frontLeftPower * max_speed);
            telemetry.addData("Back Left Motor", backLeftPower * max_speed);
            telemetry.addData("Front Right Motor", frontRightPower * max_speed);
            telemetry.addData("Back Right Motor", backRightPower * max_speed);
            telemetry.update();
        }
    }
}
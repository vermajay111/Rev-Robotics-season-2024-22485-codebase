package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentricMecnaum")
public class FieldCentricTwo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        Servo DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;
        double max_speed = 1.0;
        while (opModeIsActive()) {
            double up = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if(gamepad1.dpad_down && max_speed >= 0){
                max_speed -= 0.1;
            } else if(gamepad1.dpad_up && max_speed <= 1.0 ){
                max_speed += 0.1;
            }

           if (gamepad1.a) {
               imu = hardwareMap.get(IMU.class, "imu");
               parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                       RevHubOrientationOnRobot.LogoFacingDirection.UP,
                       RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
               imu.initialize(parameters);
               imu.resetYaw();
               double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }


            if (gamepad1.y) {
                DroneLauncher.setPosition(0);
            } else if (gamepad1.b) {
                DroneLauncher.setPosition(1);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - up * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + up * Math.cos(-botHeading);


            //rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator) * max_speed;
            double backLeftPower = ((rotY - rotX + rx) / denominator) * max_speed;
            double frontRightPower = ((rotY - rotX - rx) / denominator) * max_speed;
            double backRightPower = ((rotY + rotX - rx) / denominator) * max_speed;


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("IMU X", botHeading);
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("backLeftMotor", backLeftPower);
            telemetry.addData("frontRightMotor", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("Max_Speed", max_speed);

            telemetry.update();

        }
    }
}
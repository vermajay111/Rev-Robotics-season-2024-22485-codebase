package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field_Centric_Mecnaum")
public class FieldCentricMecnaumDrive extends LinearOpMode {
    private DistanceSensor distanceSensorFront;
    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor =  hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        Servo DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
        Servo IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive())  {

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double up = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

           if (gamepad1.a) {
               imu = hardwareMap.get(IMU.class, "imu");
               parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                       RevHubOrientationOnRobot.LogoFacingDirection.UP,
                       RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
               imu.initialize(parameters);
               imu.resetYaw();
               botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            if (gamepad1.y) {
                DroneLauncher.setPosition(0);
            } else if (gamepad1.b) {
                DroneLauncher.setPosition(1);
            }

            if (gamepad1.left_bumper){
                IntakeServo.setPosition(0);
            } else if (gamepad1.right_bumper){
                for (int i = 0; i < 15; i ++){
                    frontLeftMotor.setPower(0.3);
                    backLeftMotor.setPower(0.3);
                    frontRightMotor.setPower(0.3);
                    backRightMotor.setPower(0.3);
                    telemetry.update();
                }
                IntakeServo.setPosition(1);
            }

            if (gamepad1.dpad_up){
                up = 0.5;
            } if (gamepad1.dpad_right){
                x = 0.5;
            } if (gamepad1.dpad_left){
                x = -0.5;
            } if (gamepad1.dpad_down){
                up = -0.5;
            }

            double rotX = x * Math.cos(-botHeading) - up * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + up * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator);
            double backLeftPower = ((rotY - rotX + rx) / denominator);
            double frontRightPower = ((rotY - rotX - rx) / denominator );
            double backRightPower = ((rotY + rotX - rx) / denominator);
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (botHeading == 0.0){
                telemetry.addLine("Reset IMU");
            }
            telemetry.update();

        }
    }
}
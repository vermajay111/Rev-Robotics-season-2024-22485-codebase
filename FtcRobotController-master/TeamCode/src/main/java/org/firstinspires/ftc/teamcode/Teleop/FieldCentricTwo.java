package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricTwo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");

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

        while (opModeIsActive()) {
            double up = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

           if (gamepad1.a) {
               imu = hardwareMap.get(IMU.class, "imu");
               parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                       RevHubOrientationOnRobot.LogoFacingDirection.UP,
                       RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
               imu.initialize(parameters);
               imu.resetYaw();
               double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
               telemetry.addData("trggired", "Yes");
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - up * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + up * Math.cos(-botHeading);


            //rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;




            frontLeftMotor.setPower(frontLeftPower * 0.9);
            backLeftMotor.setPower(backLeftPower * 0.9);
            frontRightMotor.setPower(frontRightPower * 0.9);
            backRightMotor.setPower(backRightPower * 0.9);

            telemetry.addData("IMU X", botHeading);
            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("backLeftMotor", backLeftPower);
            telemetry.addData("frontRightMotor", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);

            telemetry.update();



        }
    }
}
package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecnaum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRight = hardwareMap.dcMotor.get("rightRear");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


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
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }


            //alt math to double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading); if the other one is wrong

            double gyroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double temp = drive * Math.cos(gyroAngle) + strafe * Math.sin(gyroAngle);

            strafe = -drive * Math.sin(gyroAngle) + strafe * Math.cos(gyroAngle);
            drive = temp;


            double flPower = drive + rotate + strafe;
            double frPower = drive - rotate - strafe;
            double blPower = drive + rotate - strafe;
            double brPower = drive - rotate + strafe;

            double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));

            if (maxPower > 1) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            telemetry.addData("Front Left Power", flPower);
            telemetry.addData("Front Right Power", frPower);
            telemetry.addData("Back Left Power", blPower);
            telemetry.addData("Back Right Power", brPower);
            telemetry.update();
        }
    }

}



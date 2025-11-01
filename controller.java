package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "controller")
public class controller extends LinearOpMode {
    @Override
    public void runOpMode() {

        // Gamepad tracking for toggles
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Motors
        DcMotor TopLeftMotor, BottomLeftMotor, TopRightMotor, BottomRightMotor;
        DcMotor Collector, AssistantShooter;
        DcMotorEx shooterMotor;
        IMU imu;

        // Initialize hardware
        TopLeftMotor = hardwareMap.get(DcMotor.class, "TopLeftMotor");
        BottomLeftMotor = hardwareMap.get(DcMotor.class, "BottomLeftMotor");
        TopRightMotor = hardwareMap.get(DcMotor.class, "TopRightMotor");
        BottomRightMotor = hardwareMap.get(DcMotor.class, "BottomRightMotor");
        Collector = hardwareMap.get(DcMotor.class, "Collector");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        AssistantShooter = hardwareMap.get(DcMotor.class, "AssistantShooter");

        imu = hardwareMap.get(IMU.class, "imu");

        // Motor directions
        TopLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        Collector.setDirection(DcMotor.Direction.REVERSE);

        // IMU setup
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);     //FORWARDS somewhat works
        imu.initialize(new IMU.Parameters(orientation));

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double MAX_POWER = -1;
        double NO_POWER = 0;
        boolean intakeToggle = false;
        double PER_REV = 28;

        while (opModeIsActive()) {
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // Intake toggle
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                intakeToggle = !intakeToggle;
            }
            shooterMotor.setPower(intakeToggle ? 0.99 : 0);

            // Shooter
            shooterMotor.setPower(gamepad2.y ? MAX_POWER : NO_POWER);

            // Assistant shooter
            AssistantShooter.setPower(gamepad2.a ? -1 : 0);

            // Drive control
            double rotate = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double r = Math.hypot(strafe, forward);
            double theta = Math.atan2(forward, strafe) - heading;

            double newForward = r * Math.sin(theta);
            double newStrafe = r * Math.cos(theta);

            TopLeftMotor.setPower(newForward + rotate + newStrafe);
            BottomLeftMotor.setPower(newForward + rotate - newStrafe);
            TopRightMotor.setPower(newForward - rotate - newStrafe);
            BottomRightMotor.setPower(newForward - rotate + newStrafe);



            double velocity = shooterMotor.getVelocity(); // ticks/second
            double shooterRPM = (velocity / PER_REV) * 60;


            telemetry.addData("Shooter Power", "%.2f", shooterMotor.getPower());
            telemetry.addData("Shooter RPM", "%.0f", shooterRPM);
            telemetry.update();
        }
    }
}

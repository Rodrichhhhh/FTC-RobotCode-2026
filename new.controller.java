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
        DcMotorEx ShooterMotor;
        IMU imu;

        // Initialize hardware
        TopLeftMotor = hardwareMap.get(DcMotor.class, "TopLeftMotor");
        BottomLeftMotor = hardwareMap.get(DcMotor.class, "BottomLeftMotor");
        TopRightMotor = hardwareMap.get(DcMotor.class, "TopRightMotor");
        BottomRightMotor = hardwareMap.get(DcMotor.class, "BottomRightMotor");
        Collector = hardwareMap.get(DcMotor.class, "Collector");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
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

        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double MAX_POWER = -0.90;
        double NO_POWER = 0;
        double PER_REV = 28;
        boolean intakeToggle = false;    //Shooter
        boolean intakeToggle2 = false;   //Assistant Shooter
        boolean intakeToggle3 = false;   //Collector

        while (opModeIsActive()) {
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // Intake toggle
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                intakeToggle = !intakeToggle;
            }
            ShooterMotor.setPower(intakeToggle ? MAX_POWER : NO_POWER);

            //assistant shooter
            if (currentGamepad2.a && !previousGamepad2.a) {
                intakeToggle2 = !intakeToggle2;
            }
            AssistantShooter.setPower(intakeToggle2 ? MAX_POWER : NO_POWER);


            //collector
            if (currentGamepad2.y && !previousGamepad2.y) {
                intakeToggle3 = !intakeToggle3;

            } else {
                Collector.setPower((0));
            }
            Collector.setPower(intakeToggle3 ? MAX_POWER : NO_POWER);
            
            

            // Drive control
            double rotate = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double r = Math.hypot(strafe, forward);
            double theta = Math.atan2(forward, strafe) - heading;

            //Crazy Math
            double newForward = r * Math.sin(theta);
            double newStrafe = r * Math.cos(theta);

            TopLeftMotor.setPower(newForward + rotate + newStrafe);
            BottomLeftMotor.setPower(newForward + rotate - newStrafe);
            TopRightMotor.setPower(newForward - rotate - newStrafe);
            BottomRightMotor.setPower(newForward - rotate + newStrafe);


            double velocity = ShooterMotor.getVelocity(); // ticks/second
            double shooterRPM = (velocity / PER_REV) * 60;

            //RPM display calculations
            telemetry.addData("Shooter Power", "%.2f", ShooterMotor.getPower());
            telemetry.addData("Shooter RPM", "%.0f", shooterRPM);
            telemetry.update();
        }
    }
}

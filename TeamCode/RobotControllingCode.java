package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import  com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "controller")
public class controller extends LinearOpMode {
    @Override
    public void runOpMode() {
        //set up motors and variables
        DcMotor TopLeftMotor, BottomLeftMotor, TopRightMotor, BottomRightMotor, Collector, AssistantShooter;
        DcMotorEx shooterMotor;
        IMU imu;


        TopLeftMotor = hardwareMap.get(DcMotor.class, "TopLeftMotor");
        BottomLeftMotor = hardwareMap.get(DcMotor.class, "BottomLeftMotor");
        TopRightMotor = hardwareMap.get(DcMotor.class, "TopRightMotor");
        BottomRightMotor = hardwareMap.get(DcMotor.class, "BottomRightMotor");
        Collector = hardwareMap.get(DcMotor.class, "Collector");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        AssistantShooter = hardwareMap.get(DcMotor.class, "AssistantShooter");
        imu = hardwareMap.get(IMU.class, "imu");


        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        TopLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        BottomRightMotor.setDirection(DcMotor.Direction.REVERSE);


        Collector.setDirection(DcMotor.Direction.REVERSE);


        TopLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        double PER_REV = 28;
        //double HALF_POWER = -0.5
        double MAX_POWER = -1;
        double NO_POWER = 0;
        //double SevenFive_Power = -1;


        while (opModeIsActive()) {

            // COLLECTOR
            if (gamepad2.y) {
                Collector.setPower(-1);  // adjust speed if needed

            } else {
                Collector.setPower(0);  // stop when button released
            }


            // SHOOTER
            if (gamepad2.right_bumper) {
                shooterMotor.setPower(MAX_POWER);


            } else {
                shooterMotor.setPower(NO_POWER); // stop shooter


            }

            //if (gamepad2.left_bumper) {
            //shooterMotor.setPower(HALF_POWER);
            //} else {
            //shooterMotor.setPower(NO_POWER); // stop shooter

            // }


            double velocity = shooterMotor.getVelocity(); // ticks/second
            double shooterRPM = (velocity / PER_REV) * 60;

            telemetry.addData("Shooter Power", "%.2f", shooterMotor.getPower());
            telemetry.addData("Shooter RPM", "%.0f", shooterRPM);
            telemetry.update();

            //AssistantShooter

            if (gamepad2.a) {
                AssistantShooter.setPower(-1);
            } else {
                AssistantShooter.setPower(0);
            }

            //MOVING
            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

            imu.initialize(new IMU.Parameters(RevOrientation));

            waitForStart();

            while (opModeIsActive()) {
                double rotate = gamepad1.right_stick_x;
                double strafe = gamepad1.left_stick_x;
                double forward = -gamepad1.left_stick_y;

                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Field-centric transformation
                double theta = Math.atan2(forward, strafe);
                double r = Math.hypot(strafe, forward);
                theta -= heading;

                double newForward = r * Math.sin(theta);
                double newStrafe = r * Math.cos(theta);

                TopLeftMotor.setPower(newForward + rotate + newStrafe);
                BottomLeftMotor.setPower(newForward + rotate - newStrafe);
                TopRightMotor.setPower(newForward - rotate - newStrafe);
                BottomRightMotor.setPower(newForward - rotate + newStrafe);
            }

        }

    }

}

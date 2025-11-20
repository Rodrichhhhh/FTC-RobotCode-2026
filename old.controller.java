package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.IMU;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
        //IMU imu;

        // Initialize hardware
        TopLeftMotor = hardwareMap.get(DcMotor.class, "TopLeftMotor");
        BottomLeftMotor = hardwareMap.get(DcMotor.class, "BottomLeftMotor");
        TopRightMotor = hardwareMap.get(DcMotor.class, "TopRightMotor");
        BottomRightMotor = hardwareMap.get(DcMotor.class, "BottomRightMotor");
        Collector = hardwareMap.get(DcMotor.class, "Collector");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        AssistantShooter = hardwareMap.get(DcMotor.class, "AssistantShooter");

        //imu = hardwareMap.get(IMU.class, "imu");

        // Motor directions
        TopLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        Collector.setDirection(DcMotor.Direction.REVERSE);

        // IMU setup
//        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);     //FORWARDS somewhat works
//        imu.initialize(new IMU.Parameters(orientation));
//
//        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double MAX_POWER = 1;           //Max power (Used by the shooter)
        double QUARTER_POWER = -0.75;    //power used by collector Assistant shooter
        double HALF_POWER = -0.5;        //power used by Assistant shooter
        double NO_POWER = 0;             //power of motors when inactive
        double PER_REV = 28;             //REV rotations
        boolean intakeToggle = false;    //Shooter

        while (opModeIsActive()) {
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // Intake toggle
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                intakeToggle = !intakeToggle;
            }
            ShooterMotor.setPower(intakeToggle ? MAX_POWER : NO_POWER);

            //assistant shooter
            if (gamepad2.b) {
                AssistantShooter.setPower(HALF_POWER);
            }

            else {
                AssistantShooter.setPower(NO_POWER);
            }



            //collector
            if (gamepad2.y) {
                Collector.setPower(QUARTER_POWER);
            }
            else {
                Collector.setPower(NO_POWER);
            }


            // Drive control
            double rotate = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;

//           double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            double r = Math.hypot(strafe, forward);
//            double theta = Math.atan2(forward, strafe) - heading;
//
//            //Crazy Math
//            double newForward = r * Math.sin(theta);
//            double newStrafe = r * Math.cos(theta);

            TopLeftMotor.setPower(forward + rotate + strafe);
            BottomLeftMotor.setPower(forward + rotate - strafe);
            TopRightMotor.setPower(forward - rotate - strafe);
            BottomRightMotor.setPower(forward - rotate + strafe);


            double velocity = ShooterMotor.getVelocity();   // ticks/second
            double shooterRPM = -(velocity / PER_REV) * 60;
            double rpmLimit = 3000;

            //RPM display calculations
            telemetry.addData("Shooter Power", "%.2f", ShooterMotor.getPower());
            telemetry.addData("Shooter RPM", "%.0f", shooterRPM);
            telemetry.update();

            if (shooterRPM > rpmLimit) {
                gamepad2.rumble(1.0, 1.0, 300);

            }

        }
    }
}

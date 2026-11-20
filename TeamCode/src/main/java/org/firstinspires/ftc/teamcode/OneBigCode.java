package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Bench;
import org.firstinspires.ftc.teamcode.mechanisms.BigWheel;
import org.firstinspires.ftc.teamcode.mechanisms.UpDownVroom;

@TeleOp(name = "One Big Code", group = "Robot")
public class OneBigCode extends OpMode {
    // Drivetrain motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // IMU for field-centric driving
    private IMU imu;

    // Mechanisms
    private final Bench bench = new Bench();
    private final UpDownVroom upDown = new UpDownVroom();
    private final BigWheel bigWheel = new BigWheel();

    @Override
    public void init() {
        // Initialize drivetrain motors with fallbacks for alternative naming conventions
        frontLeftDrive = getDcMotor("frontLeftDrive", "Front_Left", "front_left_motor", "FL");
        frontRightDrive = getDcMotor("frontRightDrive", "Front_Right", "front_right_motor", "FR");
        backLeftDrive = getDcMotor("backLeftDrive", "Back_Left", "back_left_motor", "BL");
        backRightDrive = getDcMotor("backRightDrive", "Back_Right", "back_right_motor", "BR");

        // Reverse left motors
        if (backLeftDrive != null) {
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (frontLeftDrive != null) {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (frontRightDrive != null) {
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (backRightDrive != null) {
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize IMU safely
        try {;
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

            RevHubOrientationOnRobot orientationOnRobot = new
                    RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        } catch (Exception e) {
            imu = null;
        }

        // Initialize mechanisms
        bench.init(hardwareMap);
        upDown.init(hardwareMap);
        bigWheel.init(hardwareMap);
    }

    @Override
    public void loop() {
        // --- Drivetrain Logic (Gamepad 1) ---
        if (gamepad1.a && imu != null) {
            imu.resetYaw();
        }

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.left_bumper || imu == null) {
            drive(forward, right, rotate);
        } else {
            driveFieldRelative(forward, right, rotate);
        }

        // --- Mechanism Logic (Gamepad 2) ---
        // CONTROLLER MAP (gamepad 2) -- hold the button, let go = stop
        //   A  = servo_rot forward        B  = servo_rot back
        //   X  = servo_feed forward       Y  = big wheel forward
        //   LB = silly forward            RB = silly back
        //   LT = UpAndDown negative       RT = UpAndDown positive
        // Change this map with the team before changing the code below.

        // Bench Motor ("silly"): LB = forward, RB = back
        if (gamepad2.left_bumper) {
            bench.setMotorSpeed(-1.0);   // same direction A used to run it
        } else if (gamepad2.right_bumper) {
            bench.setMotorSpeed(1.0);
        } else {
            bench.setMotorSpeed(0);
        }

        // Bench Servo ("servo_rot"): A = forward, B = back
        if (gamepad2.a) {
            bench.setServoRot(1.0);
        } else if (gamepad2.b) {
            bench.setServoRot(-1.0);
        } else {
            bench.setServoRot(0);
        }

        // UpDown Motor ("UpAndDown")
        if (gamepad2.left_trigger > 0.1) {
            upDown.setMotorSpeed(-0.3);
        } else if (gamepad2.right_trigger > 0.1) {
            upDown.setMotorSpeed(0.3);
        } else {
            upDown.setMotorSpeed(0);
        }

        // BigWheel Motor ("massive_wheel")
        // Hold Y = forward, let go = stop. No reverse. (If it spins the wrong way, change 1.0 to -1.0.)
        if (gamepad2.y) {
            bigWheel.setMotorSpeed(1.0);
            telemetry.addData("BigWheel", "ON");
        } else {
            bigWheel.setMotorSpeed(0);
            telemetry.addData("BigWheel", "OFF");
        }

        // Bench Feed Servo ("servo_feed")
        // Hold X = feed, let go = back to rest. Tune the numbers at the top of Bench.java.
        bench.setServoFeed(gamepad2.x);
        telemetry.addData("servo_feed", "%s  %s", bench.servoFeedStatus(), gamepad2.x ? "FEEDING" : "rest");

            // --- Telemetry ---
        telemetry.addData("Status", "Running");
        if (imu != null) {
            telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Heading", "IMU unavailable");
        }
        telemetry.addData("Encoders", "FL:%d FR:%d BL:%d BR:%d",
                frontLeftDrive != null ? frontLeftDrive.getCurrentPosition() : 0,
                frontRightDrive != null ? frontRightDrive.getCurrentPosition() : 0,
                backLeftDrive != null ? backLeftDrive.getCurrentPosition() : 0,
                backRightDrive != null ? backRightDrive.getCurrentPosition() : 0);
        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        if (imu == null) {
            drive(forward, right, rotate);
            return;
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the joystick vector by -heading to convert field -> robot coordinates
        double rotRight = right * Math.cos(heading) + forward * Math.sin(heading);
        double rotForward = -right * Math.sin(heading) + forward * Math.cos(heading);

        drive(rotForward, rotRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double Front_LeftPower = forward + right + rotate;
        double Front_RightPower = forward - right - rotate;
        double Back_RightPower = forward + right - rotate;
        double Back_LeftPower = forward - right + rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(Front_LeftPower));
        maxPower = Math.max(maxPower, Math.abs(Front_RightPower));
        maxPower = Math.max(maxPower, Math.abs(Back_RightPower));
        maxPower = Math.max(maxPower, Math.abs(Back_LeftPower));

        if (frontLeftDrive != null) frontLeftDrive.setPower(Front_LeftPower / maxPower);
        if (frontRightDrive != null) frontRightDrive.setPower(Front_RightPower / maxPower);
        if (backLeftDrive != null) backLeftDrive.setPower(Back_LeftPower / maxPower);
        if (backRightDrive != null) backRightDrive.setPower(Back_RightPower / maxPower);
    }

    private DcMotor getDcMotor(String... names) {
        for (String name : names) {
            try {
                DcMotor motor = hardwareMap.get(DcMotor.class, name);
                if (motor != null) return motor;
            } catch (Exception ignored) {
            }
        }
        return null;
    }
}


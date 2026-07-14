package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Emerson_TeleOp", group = "Competition")
public class Emerson_TeleOp extends OpMode {

    // Drivetrain motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // Mechanism motors
    private DcMotorEx launcher;
    private DcMotor intake1;
    private DcMotor intake2;

    // Servo
    private Servo armServo;

    // Servo positions
    private static final double ARM_UP_POSITION = 1.0;
    private static final double ARM_DOWN_POSITION = 0.0;

    @Override
    public void init() {

        /*
         * These names must exactly match the names in the
         * Driver Station robot configuration.
         */
        frontLeftDrive =
                hardwareMap.get(DcMotor.class, "leftFront");

        frontRightDrive =
                hardwareMap.get(DcMotor.class, "rightFront");

        backLeftDrive =
                hardwareMap.get(DcMotor.class, "leftBack");

        backRightDrive =
                hardwareMap.get(DcMotor.class, "rightBack");

        launcher =
                hardwareMap.get(DcMotorEx.class, "launcher");

        intake1 =
                hardwareMap.get(DcMotor.class, "intake1");

        intake2 =
                hardwareMap.get(DcMotor.class, "intake2");

        armServo =
                hardwareMap.get(Servo.class, "armServo");

        /*
         * Motor directions may need to be changed depending
         * on how the motors are mounted.
         */
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Brake mode prevents the drivetrain from freely rolling
         * when the joysticks are released.
         */
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * Configure the launcher to use its encoder.
         */
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients launcherPIDF =
                new PIDFCoefficients(
                        50.0,
                        0.75,
                        1.0,
                        12.7
                );

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                launcherPIDF
        );

        /*
         * Starting servo position.
         */
        armServo.setPosition(ARM_DOWN_POSITION);

        telemetry.addLine("Robot initialized");
        telemetry.addLine("Press Play to begin");
        telemetry.update();
    }

    @Override
    public void loop() {

        driveRobot();
        controlLauncher();
        controlIntake();
        controlArmServo();
        showTelemetry();
    }

    private void driveRobot() {

        /*
         * FTC joysticks return negative values when pushed forward,
         * so gamepad1.left_stick_y is negated.
         */
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        /*
         * Mecanum wheel calculations.
         */
        double frontLeftPower =
                forward + strafe + rotate;

        double frontRightPower =
                forward - strafe - rotate;

        double backLeftPower =
                forward - strafe + rotate;

        double backRightPower =
                forward + strafe - rotate;

        /*
         * Normalize motor powers so none exceed 1.0.
         */
        double largestPower = Math.max(
                Math.abs(frontLeftPower),
                Math.max(
                        Math.abs(frontRightPower),
                        Math.max(
                                Math.abs(backLeftPower),
                                Math.abs(backRightPower)
                        )
                )
        );

        if (largestPower > 1.0) {
            frontLeftPower /= largestPower;
            frontRightPower /= largestPower;
            backLeftPower /= largestPower;
            backRightPower /= largestPower;
        }

        /*
         * Hold the left trigger for slower, more precise driving.
         */
        double speedMultiplier;

        if (gamepad1.left_trigger > 0.25) {
            speedMultiplier = 0.4;
        } else {
            speedMultiplier = 1.0;
        }

        frontLeftDrive.setPower(
                frontLeftPower * speedMultiplier
        );

        frontRightDrive.setPower(
                frontRightPower * speedMultiplier
        );

        backLeftDrive.setPower(
                backLeftPower * speedMultiplier
        );

        backRightDrive.setPower(
                backRightPower * speedMultiplier
        );
    }

    private void controlLauncher() {

        /*
         * The right trigger controls launcher power.
         */
        double launcherPower =
                Range.clip(gamepad1.right_trigger, 0.0, 1.0);

        launcher.setPower(launcherPower);
    }

    private void controlIntake() {

        /*
         * Right bumper: intake forward
         * Left bumper: intake reverse
         */
        double intakePower;

        if (gamepad1.right_bumper) {
            intakePower = 1.0;
        } else if (gamepad1.left_bumper) {
            intakePower = -1.0;
        } else {
            intakePower = 0.0;
        }

        intake1.setPower(intakePower);
        intake2.setPower(intakePower);
    }

    private void controlArmServo() {

        /*
         * A raises the arm.
         * B lowers the arm.
         */
        if (gamepad1.a) {
            armServo.setPosition(ARM_UP_POSITION);
        }

        if (gamepad1.b) {
            armServo.setPosition(ARM_DOWN_POSITION);
        }
    }

    private void showTelemetry() {

        telemetry.addData(
                "Launcher Power",
                "%.2f",
                launcher.getPower()
        );

        telemetry.addData(
                "Launcher Velocity",
                "%.1f ticks/sec",
                launcher.getVelocity()
        );

        telemetry.addData(
                "Intake Power",
                "%.2f",
                intake1.getPower()
        );

        telemetry.addData(
                "Arm Position",
                "%.2f",
                armServo.getPosition()
        );

        telemetry.addLine("");
        telemetry.addLine("Left stick: drive/strafe");
        telemetry.addLine("Right stick: turn");
        telemetry.addLine("Left trigger: slow drive");
        telemetry.addLine("Right trigger: launcher");
        telemetry.addLine("Right bumper: intake");
        telemetry.addLine("Left bumper: reverse intake");
        telemetry.addLine("A/B: move arm servo");

        telemetry.update();
    }

    @Override
    public void stop() {

        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);

        launcher.setPower(0.0);
        intake1.setPower(0.0);
        intake2.setPower(0.0);
    }
}
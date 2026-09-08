package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp
public class TeleOpTurretTest extends OpMode {

    // Creates variables to store the four drivetrain motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor flywheel;
    private DcMotor flywheel2;
    private CRServo adaptorServo;
    private Limelight3A limelight;
    double previousError = 0.0;
    double integral = 0.0;
    long previousTime = System.nanoTime();
    double lastTimestamp = -1;
    private static final int TARGET_TAG_ID = 20;
    private static final double P_COEFF = 0.025;
    private static final double I_COEFF = 0.0;
    private static final double D_COEFF = 0.000;

    private static final double MIN_POWER = 0.00;
    private static final double ALLOWABLE_ERROR_DEG = 0.75;
    private static final double MAX_INTEGRAL = 100.0;

    @Override
    public void start() { // Runs once when PLAY is pressed
        limelight.start();
        previousTime = System.nanoTime();
    }

    @Override
    public void stop() {
        adaptorServo.setPower(0.0);
        limelight.stop();
    }

    @Override
    public void init() {
        { // Runs once when you press INIT on the Driver Station

            // Connects each Java motor variable to the motor name in the Robot Configs
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); // the second part where it says "frontLeft" is the name in configs
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            intake1 = hardwareMap.get(DcMotor.class, "intake1");
            intake2 = hardwareMap.get(DcMotor.class, "intake2");

            // Reverses the left side motors because of how the wheels are placed
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);


            // Makes motors stop smoothly instead of freely spinning when joystick is released
            // Last season we just stopped the power and let it stop by itself (it wasn't a problem, but I'm adding this because I saw it on discord)
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Turret
            flywheel = hardwareMap.get(DcMotor.class, "flywheel");
            flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

            adaptorServo = hardwareMap.get(CRServo.class, "adaptorServo");
            adaptorServo.setPower(0.0);

            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);


            telemetry.addData("Status", "Initialized. Target Tag: " + TARGET_TAG_ID);
            telemetry.update();
        }

    }

    @Override
    public void loop() { // Runs repeatedly throughout the whole time it is running.

        // Gets controller joystick inputs
        double y = -gamepad1.left_stick_y;//forward and backward

        double x = gamepad1.left_stick_x; //strafe

        double rx = gamepad1.right_stick_x;//rotate



        // Calculates the power needed for each mecanum wheel
        double frontLeftPower = y + x + rx;
        //checks inputs and decides action for each wheel

        double backLeftPower = y - x + rx;

        double frontRightPower = y - x - rx;

        double backRightPower = y + x - rx;


        //Checks the absolute value of all the power values and outputs the highest one
        double max = Math.max(
                Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))
        );


        // Makes sure no motor power is above 1.0 because the highest value for motors is 1
        // Keeps the same movement direction but lowers speed if power is higher the 1
        if (max > 1) {

            frontLeftPower /= max; // Scales front left power down || Max is the highest abs found above
            backLeftPower /= max; // Scales back left power down
            frontRightPower /= max; // Scales front right power down
            backRightPower /= max; // Scales back right power down
        }



        // Sends the calculated power values to the motors
        frontLeft.setPower(frontLeftPower * 2.0/3);
        frontRight.setPower(frontRightPower * 2.0/3);
        backLeft.setPower(-backLeftPower * 2.0/3);
        backRight.setPower(-backRightPower * 2.0/3);

        if (gamepad1.right_bumper) {
            intake1.setPower(1);
            intake2.setPower(-1);

        } else if (gamepad1.left_bumper) {
            intake1.setPower(-1);
            intake2.setPower(1);


        } else {
            intake1.setPower(0.25);
            intake2.setPower(0);
        }

        if (gamepad1.right_trigger_pressed){

            flywheel.setPower(gamepad1.right_trigger);
            flywheel2.setPower(gamepad1.right_trigger);
        } else {
            flywheel.setPower(0);
            flywheel2.setPower(0);
        }
        //---------------------------------------------Turret---------------------------------------------\\
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            return;
        }

        if (result.getTimestamp() == lastTimestamp) {
            return;              // same frame — leave the servo alone
        }
        lastTimestamp = result.getTimestamp();

        boolean targetFound = false;

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            for (LLResultTypes.FiducialResult detection : fiducials) {

                if (detection.getFiducialId() == TARGET_TAG_ID) {

                    targetFound = true;

                    double error = detection.getTargetXDegrees();

                    long currentTime = System.nanoTime();

                    double deltaTime =
                            (currentTime - previousTime) / 1_000_000_000.0;

                    previousTime = currentTime;

                    if (deltaTime <= 0) {
                        deltaTime = 0.01;
                    }

                    double proportional = P_COEFF * error;

                    integral += error * deltaTime;

                    integral = Math.max(
                            -MAX_INTEGRAL,
                            Math.min(MAX_INTEGRAL, integral)
                    );

                    double integralTerm = I_COEFF * integral;

                    double derivative =
                            (error - previousError) / deltaTime;

                    double derivativeTerm =
                            D_COEFF * derivative;

                    previousError = error;

                    double power =
                            -(proportional
                                    + integralTerm
                                    + derivativeTerm);

                    if (Math.abs(error) <= ALLOWABLE_ERROR_DEG) {
                        power = -derivativeTerm;   // keep braking, stop chasing
                        integral = 0.0;
                    }

                    if (power > 0 &&   power < MIN_POWER) {
                        power = MIN_POWER;

                    } else if (power < 0 && power > -MIN_POWER) {
                        power = -MIN_POWER;
                    }

                    power = Math.max(
                            -0.3,
                            Math.min(0.3, power)
                    );

                    adaptorServo.setPower(power);

                    telemetry.addData(
                            "target",
                            "found"
                    );

                    telemetry.addData(
                            "error",
                            "%.2f degrees",
                            error
                    );

                    telemetry.addData(
                            "p",
                            "%.4f",
                            proportional
                    );

                    telemetry.addData(
                            "i",
                            "%.4f",
                            integralTerm
                    );

                    telemetry.addData(
                            "d",
                            "%.4f",
                            derivativeTerm
                    );

                    telemetry.addData(
                            "servo power",
                            "%.3f",
                            power
                    );

                    telemetry.update();

                    break;
                }
            }
        }

        if (!targetFound) {

            adaptorServo.setPower(0.0);

            integral = 0.0;
            previousError = 0.0;

            telemetry.addData(
                    "target",
                    "not found"
            );

            telemetry.update();
        }



    }
}
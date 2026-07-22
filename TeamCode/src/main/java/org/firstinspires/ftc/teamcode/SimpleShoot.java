package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class SimpleShoot extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake1;
    private DcMotor intake2;
    @Override

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean velocityLock = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.b){
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                flywheel.setPower(0);
                intake2.setPower(0);
                intake1.setPower(0);
                continue;
            }
            double power = gamepad1.right_trigger;
            telemetry.addData("power:",power);
            telemetry.update();
            if (gamepad1.a){
                velocityLock = !velocityLock;
                telemetry.addData("locked",velocityLock);
                telemetry.update();
            }

            if (!velocityLock)
                flywheel.setPower(power);



                // Gets controller joystick inputs
            double y = -gamepad1.left_stick_y;//forward and backward


                double rx = gamepad1.right_stick_x;//rotate

                // Calculates the power needed for each wheel
                double frontLeftPower = y  + rx;
                //checks inputs and decides action for each wheel

                double backLeftPower = y + rx;

                double frontRightPower = y - rx;

                double backRightPower = y - rx;

                //Checks the absolute value of all the power values and outputs the highest one
                double max = Math.max(
                        Math.abs(frontLeftPower),
                        Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))
                );

                // Makes sure no motor power is above 1.0 because the highest value for motors is 1
                // Keeps the same movement direction but lowers speed if power is higher than 1
                if (max > 1) {

                    frontLeftPower /= max; // Scales front left power down || Max is the highest abs found above
                    backLeftPower /= max; // Scales back left power down
                    frontRightPower /= max; // Scales front right power down
                    backRightPower /= max; // Scales back right power down
                }

                // Sends the calculated power values to the motors
                frontLeft.setPower(frontLeftPower/2);
                frontRight.setPower(frontRightPower/2);
                backLeft.setPower(-backLeftPower/2);
                backRight.setPower(-backRightPower/2);

                if (gamepad1.right_bumper) {
                    intake1.setPower(1);
                    intake2.setPower(1);

                } else if (gamepad1.left_bumper) {
                    intake1.setPower(-1);
                    intake2.setPower(-1);

                } else {
                    intake1.setPower(0.25);
                    intake2.setPower(0);
                }

        }
    }
}

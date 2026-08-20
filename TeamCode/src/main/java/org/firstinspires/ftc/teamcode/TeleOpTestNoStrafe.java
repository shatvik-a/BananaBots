package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleOpTestNoStrafe extends OpMode {

    // Creates variables to store the four drivetrain motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor launcher;

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

            launcher = hardwareMap.get(DcMotor.class, "flywheel");

            // Reverses the left side motors because of how the wheels are placed
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);


            // Makes motors stop smoothly instead of freely spinning when joystick is released
            // Last season we just stopped the power and let it stop by itself (it wasn't a problem, but I'm adding this because I saw it on discord)
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }

    @Override
    public void loop() { // Runs repeatedly throughout the whole time it is running.
        if (gamepad1.b)
             launcher.setPower(-0.65);


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
            intake2.setPower(-1);

        } else if (gamepad1.left_bumper) {
            intake1.setPower(-1);
            intake2.setPower(1);


        } else {
            intake1.setPower(0.25);
            intake2.setPower(0);
        }




    }

}
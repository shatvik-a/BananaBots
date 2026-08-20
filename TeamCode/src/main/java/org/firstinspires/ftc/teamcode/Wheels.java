package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Wheels extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void init() { // Runs once when you press INIT on the Driver Station

        // Connects each Java motor variable to the motor name in the Robot Configs
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); // the second part where it says "frontLeft" is the name in configs
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

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


    @Override
    public void loop() { // Runs repeatedly while the program is running.
        if (gamepad1.a)
            frontLeft.setPower(0.25);
        else
            frontLeft.setPower(0);
        if (gamepad1.b)
            frontRight.setPower(0.25);
        else
            frontRight.setPower(0);
        if (gamepad1.x)
            backLeft.setPower(0.25);
        else
            backLeft.setPower(0);
        if (gamepad1.y)
            backRight.setPower(0.25);
        else
            backRight.setPower(0);
    }
}




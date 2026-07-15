package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "M3_CompTeleop", group = "Competition")
public class Intake extends OpMode {
    DcMotor intake1, intake2;

    @Override
    public void init() {

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper) {
            intake1.setPower(1);
            intake2.setPower(1);

        } else if (gamepad2.left_bumper) {
            intake1.setPower(-1);
            intake2.setPower(-1);


        } else {
            intake1.setPower(0);
            intake2.setPower(0);
        }

    }

}
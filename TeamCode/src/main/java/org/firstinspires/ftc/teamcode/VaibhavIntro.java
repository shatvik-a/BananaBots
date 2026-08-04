package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;// Importing the DcMotor tells it that it is going to be used

@TeleOp
public class VaibhavIntro extends OpMode {// Public Class is telling us what the topic of the code is about
DcMotor intake1;// Stating the name of the motor
DcMotor intake2;//Stating the other name of the other motor

@Override
public void init() {
    intake1 = hardwareMap.get(DcMotor.class,"intake1");// Going to use the hardware part
    intake2 = hardwareMap.get(DcMotor.class,"intake2");//(Nicknaming is also possible)
}
@Override
public void loop() {

    if (gamepad1.right_bumper) {//If right bumper is triggered do this task
        intake1.setPower(1);// Runs the intake to intake the balls
        intake2.setPower(1);
    } else if (gamepad1.left_bumper) {//If left bumper is triggered to this task
        intake1.setPower(-1);//Runs the intake the other way to spit out the balls
        intake2.setPower(-1);
    }

    } else {
        intake1.setPower(0);//Don't run anything if nothing is pressed
        intake2.setPower(0);

}
}


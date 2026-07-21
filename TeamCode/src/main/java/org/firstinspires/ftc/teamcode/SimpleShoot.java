package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class SimpleShoot extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        waitForStart();
        while (opModeIsActive()) {
            double power = -gamepad1.right_trigger;

            flywheel.setPower(power);

            telemetry.addData("Power", power);
            telemetry.update();
        }

    }
}

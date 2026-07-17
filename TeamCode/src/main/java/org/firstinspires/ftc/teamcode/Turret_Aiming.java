package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
@TeleOp
public class Turret_Aiming extends LinearOpMode {
    private CRServo adaptorServo;
    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 20;
    private static final double P_COEFF = 0.05;
    private static final double MIN_POWER = 0.05;
    private static final double ALLOWABLE_ERROR_DEG = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        adaptorServo = hardwareMap.get(CRServo.class, "adaptorServo");
        adaptorServo.setPower(0.0);
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        telemetry.addData("Status", "Initialized. Target Tag: " + TARGET_TAG_ID);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            boolean targetFound = false;
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult detection : fiducials) {
                    if (detection.getFiducialId() == TARGET_TAG_ID) {
                        targetFound = true;
                        double error = detection.getTargetXDegrees();
                        double power = 0.0;
                        if (Math.abs(error) > ALLOWABLE_ERROR_DEG) {
                            power = -error * P_COEFF;
                            if (power > 0 && power < MIN_POWER) {
                                power = MIN_POWER;
                            } else if (power < 0 && power > -MIN_POWER) {
                                power = -MIN_POWER;
                            }
                        }
                        power = Math.max(-0.5, Math.min(0.5, power));
                        adaptorServo.setPower(power);
                        telemetry.addData("Target", "FOUND (ID " + TARGET_TAG_ID + ")");
                        telemetry.addData("Bearing Error (Tx)", error);
                        telemetry.addData("Servo Power", power);
                        break;
                    }
                }
            }
            if (!targetFound) {
                adaptorServo.setPower(0.0);
                telemetry.addData("Target", "NOT FOUND (Stopped)");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}



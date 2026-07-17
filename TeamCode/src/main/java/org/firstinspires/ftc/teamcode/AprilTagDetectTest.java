package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
@TeleOp
public class AprilTagDetectTest extends LinearOpMode {
    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 20;
    private static final double P_COEFF = 0.005;
    private static final double ALLOWABLE_ERROR_DEG = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
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
                        telemetry.addData("Target", "FOUND (ID " + TARGET_TAG_ID + ")");
                        telemetry.addData("Bearing Error (Tx)", error);
                        telemetry.update();
                        break;
                    }
                }
            }
            if (!targetFound) {
                telemetry.addData("Target", "NOT FOUND (Stopped)");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}



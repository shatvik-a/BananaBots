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

    private static final double P_COEFF = 0.0256;
    private static final double I_COEFF = 0.0005;
    private static final double D_COEFF = 0.002;

    private static final double MIN_POWER = 0.05;
    private static final double ALLOWABLE_ERROR_DEG = 1.0;
    private static final double MAX_INTEGRAL = 100.0;

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

        double previousError = 0.0;
        double integral = 0.0;
        long previousTime = System.nanoTime();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

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
                            power = 0.0;
                            integral = 0.0;
                        }

                        if (power > 0 && power < MIN_POWER) {
                            power = MIN_POWER;

                        } else if (power < 0 && power > -MIN_POWER) {
                            power = -MIN_POWER;
                        }

                        power = Math.max(
                                -0.5,
                                Math.min(0.5, power)
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

        limelight.stop();
    }
}
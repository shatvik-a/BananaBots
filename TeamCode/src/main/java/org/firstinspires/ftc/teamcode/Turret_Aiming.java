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

    private static final double P_COEFF = 0.010;

    // OLD: private static final double I_COEFF = 0.0005;
    // WHY: Bug 5. The camera image lags the real turret by tens of ms. An integral term on a
    //      laggy system keeps "pushing harder" based on old info and causes wobble. Start at 0,
    //      tune P and D, then bring I back small (~0.0001) ONLY if the turret stalls short of the tag.
    private static final double I_COEFF = 0.0;

    private static final double D_COEFF = 0.0;

    private static final double MIN_POWER = 0.05;  // TODO: measure real breakaway power with a ramp test (often 0.08-0.15)

    // OLD: private static final double ALLOWABLE_ERROR_DEG = 1.0;
    // WHY: Bug 3. One hard line at 1° means: stop at 0.9°, drift to 1.1°, kick, overshoot to -1.1°,
    //      kick back... forever. Replaced with two lines (hysteresis): park inside STOP_DEG,
    //      and don't wake up until you drift past RESUME_DEG.
    private static final double STOP_DEG = 1.0;
    private static final double RESUME_DEG = 2.0;

    private static final double MAX_INTEGRAL = 100.0;

    // NEW (Bonus): 0..1, lower = smoother/slower derivative. Camera tx jitters frame to frame,
    //      which makes raw D spiky.
    private static final double D_FILTER = 0.3;

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

        // NEW (Bug 2): the loop spins thousands of times/sec but the Limelight only makes a new
        //      frame ~30-90 times/sec. getLatestResult() hands back the SAME frame repeatedly.
        //      We remember the last frame's timestamp so we only run PID on genuinely new data,
        //      and remember the last power so we can keep sending it in between frames.
        double lastTimestamp = -1;
        double lastPower = 0.0;

        // NEW (Bug 4): when the tag is lost, previousError gets zeroed. On the first frame back,
        //      D would compute (error - 0)/dt = a fake huge spike. This flag lets us skip D once.
        boolean firstFrame = true;

        // NEW (Bug 3): are we "parked" inside the deadband? (see STOP_DEG / RESUME_DEG)
        boolean holding = false;

        // NEW (Bonus): smoothed derivative
        double filteredDeriv = 0.0;

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            boolean targetFound = false;

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult detection : fiducials) {
                    if (detection.getFiducialId() == TARGET_TAG_ID) {
                        targetFound = true;

                        // NEW (Bug 2): same camera frame as last loop? Don't re-run PID, just hold.
                        //      Without this, D reads 0 on repeat frames then spikes on a new one,
                        //      and I integrates stale data.
                        if (result.getTimestamp() == lastTimestamp) {
                            adaptorServo.setPower(lastPower);
                            break;
                        }
                        lastTimestamp = result.getTimestamp();

                        double error = detection.getTargetXDegrees();

                        long currentTime = System.nanoTime();
                        double deltaTime = (currentTime - previousTime) / 1_000_000_000.0;
                        previousTime = currentTime;
                        if (deltaTime <= 0) {
                            deltaTime = 0.01;
                        }

                        double proportional = P_COEFF * error;

                        integral += error * deltaTime;
                        integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));
                        double integralTerm = I_COEFF * integral;

                        // OLD: double derivative = (error - previousError) / deltaTime;
                        // WHY: Bug 4. On the first frame after reacquiring, previousError is 0 (reset in
                        //      the not-found branch), so this produced a phantom spike. Now D is 0 on
                        //      that first frame.
                        double derivative = firstFrame ? 0.0 : (error - previousError) / deltaTime;
                        firstFrame = false;

                        // OLD: double derivativeTerm = D_COEFF * derivative;
                        // WHY: Bonus. Raw D is noisy because tx jitters. Low-pass filter it first.
                        filteredDeriv = (1.0 - D_FILTER) * filteredDeriv + D_FILTER * derivative;
                        double derivativeTerm = D_COEFF * filteredDeriv;

                        previousError = error;

                        double power = -(proportional + integralTerm + derivativeTerm);

                        // OLD:
                        // if (Math.abs(error) <= ALLOWABLE_ERROR_DEG) {
                        //     power = 0.0;
                        //     integral = 0.0;
                        // }
                        //
                        // if (power > 0 && power < MIN_POWER) {
                        //     power = MIN_POWER;
                        // } else if (power < 0 && power > -MIN_POWER) {
                        //     power = -MIN_POWER;
                        // }
                        //
                        // WHY: Bug 3. Single hard deadband + forced minimum kick = hunting back and
                        //      forth across the 1° line. Replaced with hysteresis: enter "holding" at
                        //      STOP_DEG, leave it only past RESUME_DEG. The MIN_POWER kick now only
                        //      applies when NOT holding, so it can't fight the deadband.
                        if (holding && Math.abs(error) > RESUME_DEG) {
                            holding = false;   // drifted far enough, wake up
                        } else if (!holding && Math.abs(error) <= STOP_DEG) {
                            holding = true;    // close enough, park
                        }

                        if (holding) {
                            power = 0.0;
                            integral = 0.0;
                        } else {
                            if (power > 0 && power < MIN_POWER) {
                                power = MIN_POWER;
                            } else if (power < 0 && power > -MIN_POWER) {
                                power = -MIN_POWER;
                            }
                        }

                        power = Math.max(-0.3, Math.min(0.3, power));

                        adaptorServo.setPower(power);
                        lastPower = power;     // NEW (Bug 2): remember for repeat-frame loops

                        telemetry.addData("target", "found");
                        telemetry.addData("error", "%.2f degrees", error);
                        telemetry.addData("holding", holding);
                        telemetry.addData("dt", "%.4f s", deltaTime);
                        telemetry.addData("p", "%.4f", proportional);
                        telemetry.addData("i", "%.4f", integralTerm);
                        telemetry.addData("d", "%.4f", derivativeTerm);
                        telemetry.addData("servo power", "%.3f", power);
                        telemetry.update();
                        break;
                    }
                }
            }

            if (!targetFound) {
                adaptorServo.setPower(0.0);
                integral = 0.0;
                previousError = 0.0;

                // OLD: (nothing here - previousTime was never reset)
                // WHY: Bug 1. previousTime only updated inside the found branch. Lose the tag for
                //      3 s, find it again, and deltaTime = 3.0 on the first loop back, so
                //      integral += error * 3.0 in one shot = a big unearned kick. Restart the clock.
                previousTime = System.nanoTime();

                firstFrame = true;      // NEW (Bug 4): skip D on next frame
                holding = false;        // NEW (Bug 3): forget parked state
                filteredDeriv = 0.0;    // NEW (Bonus): clear D filter
                lastPower = 0.0;        // NEW (Bug 2): don't resume with a stale power

                telemetry.addData("target", "not found");
                telemetry.update();
            }
        }

        limelight.stop();
    }
}

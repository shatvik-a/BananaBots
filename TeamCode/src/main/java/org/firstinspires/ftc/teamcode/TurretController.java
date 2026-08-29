package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TurretController {

    private final CRServo turretServo;
    private final ElapsedTime timer = new ElapsedTime();

    //==================== PD Constants ====================

    // Increase kP if it reacts too slowly
    // Increase kD if it oscillates
    private double kP = 0.04;
    private double kD = 0.002;

    //==================== Servo Settings ====================

    private static final double MIN_POWER = 0.15;      // Minimum power needed to move servo
    private static final double MAX_POWER = 0.60;      // Max turning speed
    private static final double AIM_TOLERANCE = 0.30;  // Degrees from center
    private static final double MAX_CHANGE = 0.05;     // Smooth acceleration

    //==================== Limelight Filter ====================

    // Lower = smoother but slower
    // Higher = faster but more jittery
    private static final double FILTER = 0.25;

    //==================== Variables ====================

    private double filteredTx = 0;

    private double lastError = 0;
    private double lastTime = 0;
    private double lastOutput = 0;

    public TurretController(CRServo servo) {
        turretServo = servo;
        timer.reset();
    }

    public void update(LLResult result) {

        // No target detected
        if (result == null || !result.isValid()) {
            turretServo.setPower(0);
            return;
        }

        //==================== Smooth Limelight ====================

        filteredTx += FILTER * (result.getTx() - filteredTx);

        double error = filteredTx;

        //==================== Time ====================

        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;

        if (dt <= 0.001)
            return;

        //==================== Derivative ====================

        double derivative = (error - lastError) / dt;

        //==================== PD ====================

        double output = kP * error + kD * derivative;

        //==================== Stop when centered ====================

        if (Math.abs(error) < AIM_TOLERANCE) {
            output = 0;
        }

        //==================== Servo Deadband ====================

        if (Math.abs(error) > AIM_TOLERANCE &&
                Math.abs(output) < MIN_POWER) {

            output = Math.copySign(MIN_POWER, output);
        }

        //==================== Speed Limit ====================

        output = Range.clip(output, -MAX_POWER, MAX_POWER);

        //==================== Slew Rate Limiter ====================

        double delta = output - lastOutput;

        if (delta > MAX_CHANGE)
            output = lastOutput + MAX_CHANGE;

        if (delta < -MAX_CHANGE)
            output = lastOutput - MAX_CHANGE;

        //==================== Send Power ====================

        turretServo.setPower(output);

        //==================== Save Values ====================

        lastError = error;
        lastOutput = output;
        lastTime = currentTime;
    }
}
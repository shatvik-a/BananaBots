package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name="AprilTag GoBilda Turret Lock",group="Production")

public class Turret_Aiming extends LinearOpMode{
    private CRServo adaptorServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private static final int TARGET_TAG_ID=20;
    private static final double P_COEFF=0.02;
    private static final double MIN_POWER=0.08;
    private static final double ALLOWABLE_ERROR_DEG=1.0;
    @Override
    public void runOpMode() throws InterruptedException{
        adaptorServo=hardwareMap.get(CRServo.class,"adaptorServo");
        adaptorServo.setPower(0.0);
        aprilTagProcessor=new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);
        visionPortal=new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class,"Webcam 1")).addProcessor(aprilTagProcessor).build();
        telemetry.addData("Status","Initialized. Target Tag: "+TARGET_TAG_ID);
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            AprilTagDetection targetTag=findTargetTag(TARGET_TAG_ID);
            if(targetTag!=null&&targetTag.ftcPose!=null){
                double error=targetTag.ftcPose.bearing;
                double power=0.0;
                if(Math.abs(error)>ALLOWABLE_ERROR_DEG){
                    power=error*P_COEFF;
                    if(power>0&&power<MIN_POWER){
                        power=MIN_POWER;
                    }else if(power<0&&power>-MIN_POWER){
                        power=-MIN_POWER;
                    }
                }
                power=Math.max(-0.5,Math.min(0.5,power));
                adaptorServo.setPower(power);
                telemetry.addData("Target","FOUND");
                telemetry.addData("Bearing Error",error);
                telemetry.addData("Servo Power",power);
            }else{
                adaptorServo.setPower(0.0);
                telemetry.addData("Target","NOT FOUND (Stopped)");
            }
            telemetry.update();
        }
        visionPortal.close();
    }
    private AprilTagDetection findTargetTag(int targetId){
        List<AprilTagDetection> currentDetections=aprilTagProcessor.getDetections();
        for(AprilTagDetection detection:currentDetections){
            if(detection.metadata!=null&&detection.id==targetId){
                return detection;
            }
        }
        return null;
    }
}



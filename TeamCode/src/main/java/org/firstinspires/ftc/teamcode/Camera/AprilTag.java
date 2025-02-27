package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
public class AprilTag extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    int blue_left = 13;
    int blue_right = 11;
    int red_left = 14;
    int red_right = 16;

    int larguraDaCamera = 640;
    int alturaDaCamera = 480;

    public  AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void configuracaoCamera() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(larguraDaCamera, alturaDaCamera, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void detectTag() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == blue_left || tag.id == blue_right || tag.id == red_right || tag.id == red_left) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (tagFound) {
                telemetry.addLine("A tag de interesse está à vista!\n\nDados de localização:");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Não vejo a tag de interesse :(");
                ultimaTagVista();
            }
        } else {
            telemetry.addLine("Não vejo a tag de interesse :(");
            ultimaTagVista();
        }
    }

    public void ultimaTagVista() {
        if (tagOfInterest == null) {
            telemetry.addLine("(A tag nunca foi vista)");
        } else {
            telemetry.addLine("\nMas já vimos a tag antes; última vez vista em:");
            tagToTelemetry(tagOfInterest);
        }
    }

    public void exibirTag() {
        if (tagOfInterest != null) {
            telemetry.addLine("Instantâneo da tag:\n");
            tagToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("Nenhum instantâneo de tag disponível, ela nunca foi avistada durante o loop de inicialização :(");
        }
        telemetry.update();
    }

    void tagToTelemetry(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}

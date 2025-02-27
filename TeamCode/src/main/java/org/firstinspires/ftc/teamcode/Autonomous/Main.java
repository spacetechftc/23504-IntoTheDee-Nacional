/*package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Camera.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Camera.AprilTag;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.LiftControl;
import org.firstinspires.ftc.teamcode.Subsystem.Slides_Methods;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Autonomous.RobotActions;


import java.util.ArrayList;

@TeleOp
public class Main extends LinearOpMode {


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    @Override
    public void runOpMode() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        hw.clawIn.setPosition(Constants.GARRA_INTAKE_ABERTA);
        sleep(500);
        hw.outtake.setPower(0.75);
        hw.outtake.setTargetPosition(Constants.OUTTAKE_PRONTO_PARA_PEGAR);
        hw.servoIntakeL.setPosition(Constants.INTAKE_EM_PE);
        hw.servoIntakeR.setPosition(Constants.INTAKE_EM_PE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });


        AprilTagDetection tagOfInterest = null;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a){
                hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_ABERTA);
            } else {
                hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_FECHADA);
            }

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (!currentDetections.isEmpty()) {
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 13 || tag.id == 12 || tag.id == 16 || tag.id == 15) {
                        tagOfInterest = tag;
                        telemetry.addLine("AprilTag de interesse detectada!");
                        telemetry.addData("Tag ID:", tagOfInterest.id);
                        break;
                    }
                }
            } else {
                telemetry.addLine("Nenhuma AprilTag detectada.");
            }

            telemetry.update();
            sleep(20);
        }

        waitForStart();

        if (tagOfInterest != null) {
            switch (tagOfInterest.id) {
                case 13:
                    Actions.runBlocking(
                            BlueLeft()
                    );
                    break;
                case 12:
                    // Ação para blue_right
                    break;
                case 16:
                    Actions.runBlocking(
                            RedRight()
                    );
                    break;
                case 15:
                    // Ação para red_right
                    break;
                default:
                    // Ação padrão
                    break;
            }
        } else {
            telemetry.addLine("Nenhuma tag foi avistada durante o INIT, executando a ação padrão.");
            telemetry.update();
        }
    }

    private Action BlueLeft() {
        Pose2d initialPose = new Pose2d(33.3, 61, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        HardwareConfig hardwareConfig = new HardwareConfig(hardwareMap);
        RobotActions.Outtake outtake = new RobotActions.Outtake(hardwareConfig);
        RobotActions.Lift elevadores = new RobotActions.Lift(hardwareConfig);
        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hardwareConfig);
        RobotActions.Intake bracointake = new RobotActions.Intake(hardwareConfig);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hardwareConfig);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(58, 50), Math.toRadians(230))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(-3700),
                                outtake.realiseOut(),
                                new SleepAction(1),
                                garraOut.openClaw(),
                                new SleepAction(1),
                                outtake.grabOut(),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                .strafeToLinearHeading(new Vector2d(49,41), Math.toRadians(-90))
                .stopAndAdd(
                        new SequentialAction(
                                garraIn.openClaw(),
                                bracointake.grabIn(),
                                new SleepAction(0.5),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.realiseIn(),
                                new SleepAction(0.65),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()

                        )
                )
                .setReversed(true)
                .splineTo(new Vector2d(57, 54.5), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(-3700),
                                outtake.realiseOut(),
                                new SleepAction(1),
                                garraOut.openClaw(),
                                new SleepAction(1),
                                outtake.grabOut(),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                //2° Volta (Amostra do meio)
                .setReversed(false)
                .splineTo(new Vector2d(63, 43), Math.toRadians(270))
                .stopAndAdd(
                        new SequentialAction(
                                garraIn.openClaw(),
                                bracointake.grabIn(),
                                new SleepAction(0.5),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.realiseIn(),
                                new SleepAction(0.5),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()
                        )
                )
                .setReversed(true)
                .splineTo(new Vector2d(58, 54.5), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(-3700),
                                outtake.realiseOut(),
                                new SleepAction(1),
                                garraOut.openClaw(),
                                new SleepAction(1),
                                outtake.grabOut(),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                //3° Volta (Amostra do Canto)
                .splineTo(new Vector2d(63, 42.5),Math.toRadians(300))
                .stopAndAdd(
                        new SequentialAction(
                                garraIn.openClaw(),
                                bracointake.grabIn(),
                                new SleepAction(0.5),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.realiseIn(),
                                new SleepAction(0.5),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()
                        )
                )
                .setReversed(true)
                .splineTo(new Vector2d(58, 54.5), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(-3700),
                                outtake.realiseOut(),
                                new SleepAction(1),
                                garraOut.openClaw(),
                                new SleepAction(1),
                                outtake.grabOut(),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                .setReversed(false)
                .splineTo(new Vector2d(28, 10), Math.toRadians(180))
                .endTrajectory();

        return tab1.build();
    }

    private Action RedRight() {
        Pose2d initialPose = new Pose2d(33, 63, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        HardwareConfig hardwareConfig = new HardwareConfig(hardwareMap);
        RobotActions.Outtake outtake = new RobotActions.Outtake(hardwareConfig);
        RobotActions.Lift elevadores = new RobotActions.Lift(hardwareConfig);
        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hardwareConfig);
        RobotActions.Intake bracointake = new RobotActions.Intake(hardwareConfig);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hardwareConfig);

        TrajectoryActionBuilder tab1 = drive.mirroredActionBuilder(initialPose)
                .splineTo(new Vector2d(58, 50), Math.toRadians(230))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(-3700),
                                outtake.realiseOut(),
                                new SleepAction(1),
                                garraOut.openClaw(),
                                new SleepAction(0.5),
                                outtake.grabOut(),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                .strafeToLinearHeading(new Vector2d(49,42.5), Math.toRadians(-90))
                .stopAndAdd(
                        new SequentialAction(
                                garraIn.openClaw(),
                                bracointake.grabIn(),
                                new SleepAction(0.5),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.realiseIn(),
                                new SleepAction(0.5),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()

                        )
                )
                .setReversed(true)
                .splineTo(new Vector2d(57, 54.5), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(-3700),
                                outtake.realiseOut(),
                                new SleepAction(1),
                                garraOut.openClaw(),
                                new SleepAction(0.5),
                                outtake.grabOut(),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                //2° Volta (Amostra do meio)
                .setReversed(false)
                .splineTo(new Vector2d(63, 45), Math.toRadians(270))
                .stopAndAdd(
                        new SequentialAction(
                                garraIn.openClaw(),
                                bracointake.grabIn(),
                                new SleepAction(0.5),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.realiseIn(),
                                new SleepAction(0.5),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()
                        )
                )
                .setReversed(true)
                .splineTo(new Vector2d(58, 54.5), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(-3700),
                                outtake.realiseOut(),
                                new SleepAction(1),
                                garraOut.openClaw(),
                                new SleepAction(0.5),
                                outtake.grabOut(),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                //3° Volta (Amostra do Canto)
                .splineTo(new Vector2d(63, 43),Math.toRadians(300))
                .stopAndAdd(
                        new SequentialAction(
                                garraIn.openClaw(),
                                bracointake.grabIn(),
                                new SleepAction(0.5),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.realiseIn(),
                                new SleepAction(0.5),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()
                        )
                )
                .setReversed(true)
                .splineTo(new Vector2d(58, 54.5), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(-3700),
                                outtake.realiseOut(),
                                new SleepAction(1),
                                garraOut.openClaw(),
                                new SleepAction(0.5),
                                outtake.grabOut(),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                .setReversed(false)
                .splineTo(new Vector2d(28, 10), Math.toRadians(180))
                .endTrajectory();

        return tab1.build();
    }



}
/*
 */
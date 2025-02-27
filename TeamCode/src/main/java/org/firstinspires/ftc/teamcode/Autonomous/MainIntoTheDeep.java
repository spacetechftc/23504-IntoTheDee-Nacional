package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "MainInto", group = "Autonomous")
public class MainIntoTheDeep extends LinearOpMode {

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
        hw.outtake.setPower(0.1);
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
                    if (tag.id == 13 || tag.id == 11 || tag.id == 16 || tag.id == 15) {
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
                case 16:
                    Actions.runBlocking(
                            BlueLeft()
                    );
                    break;
                case 11:
                case 14:
                    Actions.runBlocking(
                            RedRight()
                    );
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

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI/2)
        ));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-80, 80);

        HardwareConfig hardwareConfig = new HardwareConfig(hardwareMap);
        RobotActions.Lift elevadores = new RobotActions.Lift(hardwareConfig);
        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hardwareConfig);
        RobotActions.Intake bracointake = new RobotActions.Intake(hardwareConfig);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hardwareConfig);
        RobotActions.ExtensionControl extensionControl = new RobotActions.ExtensionControl(hardwareConfig);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //Deposito
                .splineTo(new Vector2d(52.89383390875461, 51.97265295027128), Math.toRadians(220), baseVelConstraint, new ProfileAccelConstraint(-50, 80))
                .stopAndAdd(
                        new SequentialAction(
                                garraOut.closeClaw(),
                                elevadores.liftToTarget(-3700),
                                new SleepAction(0.5),
                                garraOut.openClaw(),
                                new SleepAction(1),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                //Primeiro sample
                .splineTo(new Vector2d( 48.464445832304484,43.5), Math.toRadians(270), null, baseAccelConstraint)
                .stopAndAdd(
                        new SequentialAction(
                                garraIn.openClaw(),
                                new SleepAction(0.25),
                                bracointake.grabIn(),
                                new SleepAction(0.5),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.realiseIn(),
                                new SleepAction(0.75),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()

                        )
                )
                //Deposito primeiro sample
                .strafeToLinearHeading(new Vector2d( 53.882067894113916, 50.47508719599699), Math.toRadians(230), null, baseAccelConstraint)
                .stopAndAdd(
                        new SequentialAction(
                                garraOut.closeClaw(),
                                elevadores.liftToTarget(-3700),
                                new SleepAction(0.5),
                                garraOut.openClaw(),
                                new SleepAction(1),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                //2° Amostra do meio
                .strafeToLinearHeading(new Vector2d( 60.3, 45.5), Math.toRadians(270), null, baseAccelConstraint)
                .stopAndAdd(
                        new SequentialAction(
                                garraIn.openClaw(),
                                new SleepAction(0.25),
                                bracointake.grabIn(),
                                new SleepAction(0.5),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.realiseIn(),
                                new SleepAction(0.75),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()
                        )
                )
                //Deposito segunda a mostra
                .strafeToLinearHeading(new Vector2d(52.89383390875461,  51.97265295027128), Math.toRadians(225), null, baseAccelConstraint)
                .stopAndAdd(
                        new SequentialAction(
                                garraOut.closeClaw(),
                                elevadores.liftToTarget(-3700),
                                new SleepAction(0.5),
                                garraOut.openClaw(),
                                new SleepAction(1),
                                elevadores.liftToTarget(0),
                                garraOut.openClaw()
                        )
                )
                .setTangent(Math.toRadians(230))
                .splineToLinearHeading(new Pose2d(25, 14, Math.toRadians(270)), Math.toRadians(180), null, new ProfileAccelConstraint(-250, 250))
                .endTrajectory();

        return tab1.build();
    }

    private Action RedRight() {
        Pose2d initialPose = new Pose2d(18, -61, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI/2)
        ));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-65, 20);

        TrajectoryActionBuilder ArrastarBlocosVerm = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(53, -10), Math.toRadians(270))
                .lineToY(-61)
                .lineToY(-10)
                .strafeTo(new Vector2d(63, -10))
                .setTangent(Math.toRadians(270))
                .lineToY(-61)
                .endTrajectory();

        return ArrastarBlocosVerm.build();
    }



}
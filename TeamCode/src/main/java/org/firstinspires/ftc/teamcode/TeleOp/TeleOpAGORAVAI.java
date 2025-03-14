package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Autonomous.RobotActions;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.MathUtils;
import org.firstinspires.ftc.teamcode.RoadRunner.Tuning.TuningOpModes;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class TeleOpAGORAVAI extends LinearOpMode {

    double action = 0;

    boolean currentstateClawIn = false;
    boolean previousButtonLBState = false;

    boolean currentClawOutState = false;
    boolean previousButtonRBState = false;

    boolean currentArmState= false;
    boolean previousXButtonState = false;

    double ori;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private int currentPipeline = -1; // Para evitar mudanças desnecessárias de pipeline
    private String orientacao;
    private int validDetectionCount = 0;
    double robotAngle;
    private Limelight3A limelight;


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        HardwareConfig hw = new HardwareConfig(hardwareMap);
        HardwareConfig hardwareConfig = new HardwareConfig(hardwareMap);

        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hardwareConfig);
        RobotActions.Intake bracointake = new RobotActions.Intake(hardwareConfig);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hardwareConfig);
        RobotActions.ExtensionControl extensionControl = new RobotActions.ExtensionControl(hardwareConfig);
        RobotActions.Outtake outtakeArm = new RobotActions.Outtake(hardwareConfig);
        ElapsedTime timerout = new ElapsedTime();
        double ccout = 0;
        double cctime = 2;



        telemetry.setMsTransmissionInterval(50);

        limelight.pipelineSwitch(0); // Definir um pipeline inicial
        currentPipeline = 0;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();
            limelight.start();

            while (opModeIsActive()) {

//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    validDetectionCount++;
//                    if (validDetectionCount >= 3) { // Requer 3 leituras válidas consecutivas
//                        processColorResults(result);
//                        validDetectionCount = 0; // Reseta após atualizar
//                    }
//                } else {
//                    validDetectionCount = 0;
//                    telemetry.addData("Limelight", "No valid data");
//                }
                telemetry.update();

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                TelemetryPacket packet = new TelemetryPacket();

                // updated based on gamepads

                // update running actions
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;

                dash.sendTelemetryPacket(packet);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                double power = -gamepad2.left_stick_y;
                hw.outtakeSlideL.setPower(power);
                hw.outtakeSlideR.setPower(power);

                telemetry.addData("SlidePosL", hw.outtakeSlideL.getCurrentPosition());
                telemetry.addData("SlidePosD", hw.outtakeSlideR.getCurrentPosition());



//                Pipe();


                double outtakepower = 0;

                if(gamepad2.dpad_up){
                    hw.outtake.setPower(0.75);
                } else if(gamepad2.dpad_down){
                    hw.outtake.setPower(-0.75);
                } else{
                    hw.outtake.setPower(0);
                }

//
//                if(hw.outtakeSlideL.getCurrentPosition() > 3000 && ccout == 0 && power<1 || cctime == 0){
//                    timerout.reset();
//                    hw.outtake.setPower(-1);
//                    if(timerout.seconds() > 1) {
//                        hw.outtake.setPower(0);
//                        ccout = 1;
//                        cctime =1;
//                    } else{
//                        cctime = 0;
//                    }
//                }
//                if(hw.outtakeSlideL.getCurrentPosition() < 1000 && ccout == 1 && power > 1 || cctime == 0){
//                    timerout.reset();
//                    hw.outtake.setPower(1);
//                    if(timerout.seconds() > 1) {
//                        hw.outtake.setPower(0);
//                        ccout = 0;
//                        cctime = 1;
//                    }else{
//                        cctime = 0;
//                    }
//                }




                telemetry.addData("outpower", outtakepower);




                if (gamepad2.b) {
                    action = 1;
                    Actions.runBlocking(new SequentialAction(
                            garraOut.openClaw(),
                            garraIn.closeClaw(),
                            new SleepAction(0.5)
                    ));
                    Actions.runBlocking(new SequentialAction(
                            bracointake.retract(),
                            new SleepAction(1),
                            extensionControl.extendTarget(200)
                    ));
                    Actions.runBlocking(new SequentialAction(
                            bracointake.pass(),
                            new SleepAction(0.5)

                    ));
                    Actions.runBlocking(new SequentialAction(
                            garraOut.closeClaw(),
                            new SleepAction(0.5),
                            garraIn.openClaw()
                    ));

                    action = 0;

                }








                //Braço Intake
                boolean currentXButtonState = gamepad2.x;

                if (currentXButtonState && !previousXButtonState) {
                    currentArmState = !currentArmState;
                    if (currentArmState) {
                        action = 1;
                        runningActions.add(new SequentialAction(
                                    bracointake.VertColet()
                            ));

                        action = 0;
                    } else {
                        action = 1;
                        runningActions.add(new SequentialAction(
                                bracointake.retract()
                        ));
                        action = 0;
                    }
                }

                previousXButtonState = currentXButtonState;

                //Garra Intake
                boolean currentButtonLBState = gamepad2.left_bumper;
                if (currentButtonLBState && !previousButtonLBState){
                    if(hw.clawOut.getPosition() == Constants.GARRA_OUTTAKE_FECHADA){
                        hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_ABERTA);
                    } else if (hw.clawOut.getPosition() == Constants.GARRA_OUTTAKE_ABERTA){
                        hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_FECHADA);
                    }

                }
                previousButtonLBState = currentButtonLBState;

                //Garra Outtake
                boolean currentButtonRBState = gamepad2.right_bumper;
                if (currentButtonRBState && !previousButtonRBState) {
                    if(hw.clawIn.getPosition() == Constants.GARRA_INTAKE_FECHADA){
                        hw.clawIn.setPosition(Constants.GARRA_INTAKE_ABERTA);
                    } else if (hw.clawIn.getPosition() == Constants.GARRA_INTAKE_ABERTA){
                        hw.clawIn.setPosition(Constants.GARRA_INTAKE_FECHADA);
                    }

                }
                previousButtonRBState = currentButtonRBState;

                double intakePower = gamepad2.right_stick_y;
               if(gamepad2.right_stick_y == 0){
                   hw.intakeSlide.setMotorDisable();
                   hw.intakeSlide.setPower(intakePower);
               } else if(gamepad2.right_stick_y > 0){
                   hw.intakeSlide.setMotorEnable();
                   hw.intakeSlide.setPower(intakePower);
               } else if(gamepad2.right_stick_y < 0){
                   hw.intakeSlide.setMotorEnable();
                   hw.intakeSlide.setPower(intakePower);
               }




                if(action == 1){
                   drive.rightBack.setPower(0);
                   drive.leftBack.setPower(0);
                   drive.rightFront.setPower(0);
                   drive.rightFront.setPower(0);
                }

                telemetry.addData("DifL", hw.servoIntakeL.getPosition());
                telemetry.addData("DifD", hw.servoIntakeR.getPosition());
                telemetry.update();


            }

        } else {
            throw new RuntimeException();
        }
    }

//    private void processColorResults(LLResult result) {
//        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//
//        for (LLResultTypes.ColorResult cr : colorResults) {
//            if (cr.getTargetCorners() == null || cr.getTargetCorners().size() < 2) {
//                telemetry.addData("Erro", "Não há cantos suficientes detectados!");
//                continue;
//            }
//
//            double robotAngle = cr.getTargetXDegrees();
//            double angle = calculateAngle(cr);
//            orientacao = calculateOrientation(angle); // Determina a orientação com base no ângulo
//            telemetry.addData("Ângulo", angle);
//            telemetry.addData("Orientação", orientacao);
//        }
//    }
//
//    private double calculateAngle(LLResultTypes.ColorResult cr) {
//        List<List<Double>> corners = cr.getTargetCorners();
//        if (corners == null || corners.size() < 2) {
//            return 0; // Proteção contra erros
//        }
//
//        double[] moments = calculateRawMoments(corners);
//        double mu20 = moments[0];
//        double mu02 = moments[1];
//        double mu11 = moments[2];
//
//        if (mu20 - mu02 == 0) {
//            return 0; // Evita divisão por zero
//        }
//
//        double angle = 0.5 * Math.atan2(2 * mu11, mu20 - mu02);
//        return Math.toDegrees(angle);
//    }
//
//    private double[] calculateRawMoments(List<List<Double>> corners) {
//        double m00 = 0, m10 = 0, m01 = 0, m20 = 0, m02 = 0, m11 = 0;
//
//        for (List<Double> point : corners) {
//            if (point.size() < 2) continue; // Evita erros
//
//            double x = point.get(0);
//            double y = point.get(1);
//
//            m00 += 1;
//            m10 += x;
//            m01 += y;
//            m20 += x * x;
//            m02 += y * y;
//            m11 += x * y;
//        }
//
//        if (m00 == 0) return new double[]{0, 0, 0}; // Evita divisão por zero
//
//        double cx = m10 / m00;
//        double cy = m01 / m00;
//        double mu20 = m20 / m00 - cx * cx;
//        double mu02 = m02 / m00 - cy * cy;
//        double mu11 = m11 / m00 - cx * cy;
//
//        return new double[]{mu20, mu02, mu11};
//    }
//
////    public String calculateOrientation(double angle) {
////        // Normaliza o ângulo para um intervalo de 0 a 180 graus
////        double normalizedAngle = Math.abs(angle) % 180;
////
////
////        if(gamepad1.left_bumper){
////            return "Horizontal";
////        } else if( gamepad1.right_bumper){
////            return "Vertical";
////        }
////        return "Indefinido";
////
////    }
//
//
//    private void Pipe() {
//        if (gamepad1.b) {
//            switchPipeline(1);
//
//        } else if (gamepad1.y) {
//            switchPipeline(2);
//
//        } else if (gamepad1.x) {
//            switchPipeline(0);
//
//        }
    }

//    private void switchPipeline(int pipeline) {
//        if (currentPipeline != pipeline) {
//            limelight.pipelineSwitch(pipeline);
//            currentPipeline = pipeline;
//            telemetry.addData("Pipeline Switch", "Mudando para " + pipeline);
//            sleep(100); // Pequena pausa para garantir a mudança de pipeline
//        }
//    }
//
//}

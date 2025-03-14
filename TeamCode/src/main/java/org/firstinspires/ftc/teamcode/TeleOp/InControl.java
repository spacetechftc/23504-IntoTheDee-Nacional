package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.RobotActions;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.Tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "IntakeControl", group = "TeleOp")
public class InControl extends LinearOpMode {

    private boolean currentArmState = false;
    private boolean previousXButtonState = false;
    private boolean previousButtonLBState = false;
    private boolean previousButtonRBState = false;
    private boolean triggerRPressed = false;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private Limelight3A limelight;
    private HardwareConfig hw;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hw = new HardwareConfig(hardwareMap);

        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hw);
        RobotActions.Intake bracointake = new RobotActions.Intake(hw);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hw);
        RobotActions.ExtensionControl extensionControl = new RobotActions.ExtensionControl(hw);
        RobotActions.Outtake outtakeArm = new RobotActions.Outtake(hw);

        ElapsedTime timerout = new ElapsedTime();
        double ccout = 0;
        double cctime = 2;

        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();
            limelight.start();

            while (opModeIsActive()) {
                try {
                    // Atualiza a telemetria
                    updateTelemetry(drive);

                    // Controle do chassi
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y, gamepad1.left_stick_x),
                            -gamepad1.right_stick_x
                    ));

                    // Controle dos slides de outtake
                    double slidePower = -gamepad2.left_stick_y;
                    hw.outtakeSlideL.setPower(slidePower);
                    hw.outtakeSlideR.setPower(slidePower);



                    // Ação do botão B
                    if (gamepad2.b) {
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
                    }



                    if(gamepad2.y){
                        Actions.runBlocking(new SequentialAction(
                                extensionControl.extendTLit(-200)
                        ));
                    }





                    // Controle do braço de intake
                    boolean currentXButtonState = gamepad2.x;
                    if (currentXButtonState && !previousXButtonState) {
                        currentArmState = !currentArmState;
                        if (currentArmState) {
                            runningActions.add(new SequentialAction(bracointake.VertColet()));
                        } else {
                            runningActions.add(new SequentialAction(bracointake.retract()));
                        }
                    }
                    previousXButtonState = currentXButtonState;



                    double THRESHOLD = 0.5;

                    // Verificar se o gatilho esquerdo está pressionado
                    boolean isTriggerLPressed = (gamepad2.left_trigger > THRESHOLD);

                    if (isTriggerLPressed) {
                        hw.outtake.setPower(-1);
                    } else if(gamepad2.left_bumper){
                        hw.outtake.setPower(1);
                    } else {
                        hw.outtake.setPower(0);
                    }


                    boolean isTriggerRPressed = (gamepad2.right_trigger > THRESHOLD);
                    if (isTriggerRPressed && !triggerRPressed) {
                        toggleServoPosition(hw.clawOut, Constants.GARRA_OUTTAKE_FECHADA, Constants.GARRA_OUTTAKE_ABERTA);
                    }

                    triggerRPressed = isTriggerRPressed;


//                    // Controle da garra de outtake
//                    boolean currentButtonLBState = gamepad2.left_bumper;
//                    if (currentButtonLBState && !previousButtonLBState) {
//                        toggleServoPosition(hw.clawOut, Constants.GARRA_OUTTAKE_FECHADA, Constants.GARRA_OUTTAKE_ABERTA);
//                    }
//                    previousButtonLBState = currentButtonLBState;

                    // Controle da garra de intake
                    boolean currentButtonRBState = gamepad2.right_bumper;
                    if (currentButtonRBState && !previousButtonRBState) {
                        toggleServoPosition(hw.clawIn, Constants.GARRA_INTAKE_FECHADA, Constants.GARRA_INTAKE_ABERTA);
                    }
                    previousButtonRBState = currentButtonRBState;

                    // Controle do intakeSlide
                    double intakePower = gamepad2.right_stick_y;
                    if (intakePower != 0) {
                        hw.intakeSlide.setMotorEnable();
                        hw.intakeSlide.setPower(intakePower);
                    } else {
                        hw.intakeSlide.setMotorDisable();
                    }

                    // Atualiza ações em execução
                    updateRunningActions();

                } catch (Exception e) {
                    telemetry.addData("Erro", e.getMessage());
                    telemetry.update();
                }
            }
        } else {
            throw new RuntimeException("Classe de drive não configurada corretamente.");
        }
    }

    private void toggleServoPosition(Servo servo, double pos1, double pos2) {
        if (servo.getPosition() == pos1) {
            servo.setPosition(pos2);
        } else {
            servo.setPosition(pos1);
        }
    }

    private void updateTelemetry(MecanumDrive drive) {
        telemetry.addData("SlidePosL", hw.outtakeSlideL.getCurrentPosition());
        telemetry.addData("SlidePosR", hw.outtakeSlideR.getCurrentPosition());
        telemetry.addData("ClawOut Position", hw.clawOut.getPosition());
        telemetry.addData("ClawIn Position", hw.clawIn.getPosition());
        telemetry.addData("IntakeSlide Power", hw.intakeSlide.getPower());
        telemetry.update();
    }

    private void updateRunningActions() {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();

        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
        dash.sendTelemetryPacket(packet);
    }
}
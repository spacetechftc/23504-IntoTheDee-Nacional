/*package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Autonomous.RobotActions;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.MathUtils;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpFTCLIB", group = "TeleOp")
public class TeleOpFTCLIB extends LinearOpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private boolean currentArmState = true;
    private boolean currentstate = false;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        // Motores para movimentação
        Motor frontLeft = new Motor(hardwareMap, "par1");
        Motor frontRight = new Motor(hardwareMap, "mET");
        Motor backLeft = new Motor(hardwareMap, "par0");
        Motor backRight = new Motor(hardwareMap, "perp");

        // Configuração dos motores
        frontLeft.setInverted(true);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Configuração do MecanumDrive
        MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Inicialização das posições iniciais dos servos
        hw.servoIntakeL.setPosition(Constants.INTAKE_EM_PE);
        hw.servoIntakeR.setPosition(Constants.INTAKE_EM_PE);
        hw.clawIn.setPosition(Constants.GARRA_INTAKE_ABERTA);
        hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_ABERTA);

        // Configuração dos Gamepads e do GamepadHandler
        GamepadHandler driverGamepad = new GamepadHandler(new GamepadEx(gamepad1));
        GamepadHandler operatorGamepad = new GamepadHandler(new GamepadEx(gamepad2));

        // Inicialização dos subsistemas do robô
        HardwareConfig hardwareConfig = new HardwareConfig(hardwareMap);
        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hardwareConfig);
        RobotActions.Intake bracointake = new RobotActions.Intake(hardwareConfig);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hardwareConfig);
        RobotActions.ExtensionControl extensionControl = new RobotActions.ExtensionControl(hardwareConfig);

        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        while (opModeIsActive()) {
            operatorGamepad.update();

            // Controle do movimento do robô
            double strafe = driverGamepad.getStrafe();
            double forward = driverGamepad.getForward() * 0.9;
            double turn = driverGamepad.getTurn() * 0.85;

            drive.driveRobotCentric(strafe, forward, turn);

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

            // Lê o eixo Y do joystick direito
            double rightStickY = operatorGamepad.upDown();

            hw.outtakeSlideL.setPower(rightStickY);
            hw.outtakeSlideR.setPower(rightStickY);
            double outtakepower;

            if(rightStickY < 0) {
                outtakepower = MathUtils.map(Math.abs(hw.outtakeSlideL.getCurrentPosition()), 200, 3700, -1.0, 1.0);
                hw.outtake.setPower(outtakepower);
            } else if (rightStickY > 0){
                outtakepower = -0.5;
                hw.outtake.setPower(outtakepower);
            }

            // Braço Intake usando o botão X
            if (operatorGamepad.xButtonReader.wasJustPressed()) {
                currentArmState = !currentArmState;
                if (currentArmState) {
                    runningActions.add(new SequentialAction(
                            bracointake.grabIn()
                    ));
                } else {
                    runningActions.add(new SequentialAction(
                            bracointake.realiseIn()
                    ));
                }
            }

            // Lógica de ação com botão B
            if (operatorGamepad.bButtonReader.wasJustPressed()) {
                Actions.runBlocking(new SequentialAction(
                        garraOut.openClaw(),
                        garraIn.closeClaw(),
                        new SleepAction(1.5)
                ));
                Actions.runBlocking(new ParallelAction(
                        bracointake.realiseIn(),
                        extensionControl.extendTarget(90)
                ));
                Actions.runBlocking(new SequentialAction(
                        new SleepAction(0.5),
                        garraOut.closeClaw(),
                        new SleepAction(0.5),
                        garraIn.openClaw()
                ));
            }

            // Garra Intake com Left Bumper
            if (operatorGamepad.leftBumperReader.wasJustPressed()) {
                currentstate = !currentstate;
                if (currentstate) {
                    runningActions.add(new SequentialAction(
                            garraIn.openClaw()
                    ));
                } else {
                    runningActions.add(new SequentialAction(
                            garraIn.closeClaw()
                    ));
                }
            }

            // Movimentação de extensões com botões Y e A
            if (operatorGamepad.yButtonReader.wasJustPressed()) {
                Actions.runBlocking(new SequentialAction(
                        extensionControl.extendTarget(-120)
                ));
            }
            if (operatorGamepad.aButtonReader.wasJustPressed()) {
                Actions.runBlocking(new SequentialAction(
                        extensionControl.extendTarget(90)
                ));
            }
        }
    }
}

 */
/*package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Autonomous.RobotActions;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeleOpIntoTheDeep extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor rightFront;

    private List<Action> runningActions = new ArrayList<>();

    boolean currentstate = true;
    boolean previousButtonState = false;
    boolean action = false;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareConfig hw = new HardwareConfig(hardwareMap);
        HardwareConfig hardwareConfig = new HardwareConfig(hardwareMap);
        RobotActions.Outtake outtake = new RobotActions.Outtake(hardwareConfig);
        RobotActions.Lift elevadores = new RobotActions.Lift(hardwareConfig);
        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hardwareConfig);
        RobotActions.Intake bracointake = new RobotActions.Intake(hardwareConfig);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hardwareConfig);
        RobotActions.ExtensionControl extensionControl = new RobotActions.ExtensionControl(hardwareConfig);

        leftFront = hardwareMap.get(DcMotorEx.class, "par1");
        leftBack = hardwareMap.get(DcMotorEx.class, "mET");
        rightBack = hardwareMap.get(DcMotorEx.class, "par0");
        rightFront = hardwareMap.get(DcMotorEx.class, "perp");

        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        hw.servoIntakeL.setPosition(Constants.INTAKE_EM_PE);
        hw.servoIntakeR.setPosition(Constants.INTAKE_EM_PE);
        hw.servoOuttakeL.setPosition(Constants.OUTTAKE_PRONTO_PARA_PEGAR);
        hw.servoOuttakeR.setPosition(Constants.OUTTAKE_PRONTO_PARA_PEGAR);
        hw.clawIn.setPosition(Constants.GARRA_INTAKE_ABERTA);
        hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_ABERTA);


        TelemetryPacket packet = new TelemetryPacket();

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);


            if(action){
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else{
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.b) {
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

            if (gamepad2.dpad_up) {
                action = true;
                Actions.runBlocking(new ParallelAction(
                        elevadores.liftToTarget(-3700),
                        outtake.realiseOut()
                ));
                action = false;
            } else if (gamepad2.dpad_down) {
                action = true;
                Actions.runBlocking(new ParallelAction(
                        elevadores.liftToTarget(0),
                        outtake.grabOut()
                ));
                action = false;
            }

            if (gamepad2.right_bumper) {
                action = true;
                Actions.runBlocking(new SequentialAction(
                        garraOut.openClaw(),
                        new SleepAction(1.0)
                ));
                Actions.runBlocking(new ParallelAction(
                        outtake.grabOut(),
                        elevadores.liftToTarget(0)
                ));
                action = false;
            }

            boolean currentButtonState = gamepad2.left_bumper;

            if (currentButtonState && !previousButtonState) {
                currentstate = !currentstate;
                if (currentstate) {
                    Actions.runBlocking(new SequentialAction(
                            garraIn.openClaw()
                    ));
                } else{
                    Actions.runBlocking(new SequentialAction(
                            garraIn.closeClaw()
                    ));
                }
            }

            previousButtonState = currentButtonState;

            if (gamepad2.x && hw.servoIntakeL.getPosition() != Constants.INTAKE_PRONTO_PARA_PEGAR) {
                Actions.runBlocking((new SequentialAction(
                        bracointake.grabIn()
                )));
            } else if (gamepad2.x && hw.servoIntakeL.getPosition() != Constants.INTAKE_EM_PE) {
                Actions.runBlocking((new SequentialAction(
                        bracointake.realiseIn()
                )));

            }

            if (gamepad2.y) {
                Actions.runBlocking(new SequentialAction(
                        extensionControl.extendTarget(-120)
                ));
            }
            if (gamepad2.a) {
                Actions.runBlocking((new SequentialAction(
                        extensionControl.extendTarget(90)
                )
                ));
            }
            double intakePower = gamepad2.left_stick_y;
            hw.intakeSlide.setPower(intakePower);


        }
    }
 }
 *
 */
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Autonomous.RobotActions;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.MathUtils;
import org.firstinspires.ftc.teamcode.RoadRunner.Tuning.TuningOpModes;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOpAGORAVAI", group = ("TeleOp"))
public class TeleOpAGORAVAI extends LinearOpMode {

    double action = 0;

    boolean currentstateClawIn = false;
    boolean previousButtonLBState = false;

    boolean currentClawOutState = false;
    boolean previousButtonRBState = false;

    boolean currentArmState= false;
    boolean previousXButtonState = false;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareConfig hw = new HardwareConfig(hardwareMap);
        HardwareConfig hardwareConfig = new HardwareConfig(hardwareMap);
        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hardwareConfig);
        RobotActions.Intake bracointake = new RobotActions.Intake(hardwareConfig);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hardwareConfig);
        RobotActions.ExtensionControl extensionControl = new RobotActions.ExtensionControl(hardwareConfig);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {


                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
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

                double power = gamepad2.left_stick_y;
                hw.outtakeSlideL.setPower(power);
                hw.outtakeSlideR.setPower(power);
                double outtakepower;


                if(power < 0) {
                    outtakepower = MathUtils.map(Math.abs(hw.outtakeSlideL.getCurrentPosition()), 200, 3700, -0.5, 0.5);
                    hw.outtake.setPower(outtakepower);
                } else if (power > 0){
                    outtakepower = -0.5;
                    hw.outtake.setPower(outtakepower);
                }

                if (gamepad2.b) {
                    action = 1;
                    Actions.runBlocking(new SequentialAction(
                            garraOut.openClaw(),
                            garraIn.closeClaw(),
                            new SleepAction(0.5)

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
                    action = 0;

                }

                //Braço Intake
                boolean currentXButtonState = gamepad2.x;

                if (currentXButtonState && !previousXButtonState) {
                    currentArmState = !currentArmState;
                    if (currentArmState) {
                        action = 1;
                        runningActions.add(new SequentialAction(
                                bracointake.grabIn()
                        ));
                        action = 0;
                    } else {
                        action = 1;
                        runningActions.add(new SequentialAction(
                                bracointake.realiseIn()
                        ));
                        action = 0;
                    }
                }

                previousXButtonState = currentXButtonState;

                //Garra Intake
                boolean currentButtonLBState = gamepad2.left_bumper;
                if (currentButtonLBState && !previousButtonLBState){
                    currentstateClawIn = !currentstateClawIn;
                    if (currentstateClawIn) {
                        action = 1;
                        runningActions.add(new SequentialAction(
                                garraOut.openClaw()
                        ));
                        action = 0;
                    } else{
                        action = 1;
                        runningActions.add(new SequentialAction(
                                garraOut.closeClaw()
                        ));
                        action = 0;
                    }
                }

                previousButtonLBState = currentButtonLBState;

                //Garra Outtake
                boolean currentButtonRBState = gamepad2.right_bumper;

                if (currentButtonRBState && !previousButtonRBState) {
                    currentClawOutState = !currentClawOutState;
                    if (currentClawOutState) {
                        action = 1;
                        runningActions.add(new SequentialAction(
                                garraIn.openClaw()
                        ));
                        action = 0;
                    } else{
                        action = 1;
                        runningActions.add(new SequentialAction(
                                garraIn.closeClaw()
                        ));
                        action = 0;
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



            }

        } else {
            throw new RuntimeException();
        }
    }

}

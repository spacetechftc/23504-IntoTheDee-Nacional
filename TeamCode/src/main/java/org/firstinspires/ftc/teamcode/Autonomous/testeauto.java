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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.Slides_Methods;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.opencv.core.Mat;

import java.util.Arrays;
@Autonomous
public class testeauto extends LinearOpMode {
    @Override
    public void runOpMode() {

        HardwareConfig hw = new HardwareConfig(hardwareMap);


        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_ABERTA);
            } else {
                hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_FECHADA);
            }
        }


        waitForStart();
        Actions.runBlocking(
                BlueLeft()
        );

    }

    private Action BlueLeft(){
        Pose2d initialPose = new Pose2d(38, 61, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI/2)
        ));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-50, 50);

        HardwareConfig hardwareConfig = new HardwareConfig(hardwareMap);
        RobotActions.Lift elevadores = new RobotActions.Lift(hardwareConfig);
        RobotActions.ClawOut garraOut = new RobotActions.ClawOut(hardwareConfig);
        RobotActions.Intake bracointake = new RobotActions.Intake(hardwareConfig);
        RobotActions.ClawIn garraIn = new RobotActions.ClawIn(hardwareConfig);
        RobotActions.ExtensionControl extensionControl = new RobotActions.ExtensionControl(hardwareConfig);
        RobotActions.Outtake outtake = new RobotActions.Outtake(hardwareConfig);



        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //Deposito
                .splineTo(new Vector2d(54.89383390875461, 51.97265295027128), Math.toRadians(230))
                .stopAndAdd(
                        new SequentialAction(
                                elevadores.liftToTarget(3100),
                                outtake.outtakeToTarget(2),
                                garraOut.openClaw(),
                                new SleepAction(0.3),
                                outtake.outtakeToTarget(-1),
                                elevadores.liftToTarget(0)

                        )
                )
                .strafeToLinearHeading(new Vector2d(48.5, 55), Math.toRadians(270))
                .stopAndAdd(
                        new SequentialAction(
                                extensionControl.extendTarget(-150),
                                new SleepAction(0.5),
                                garraIn.openClaw(),
                                bracointake.VertColet(),
                                new SleepAction(0.5),

                                garraOut.openClaw(),
                                garraIn.closeClaw(),
                                new SleepAction(0.5),
                                bracointake.retract(),
                                new SleepAction(1),



                                extensionControl.extendTarget(200),
                                bracointake.pass(),

                                new SleepAction(0.5),
                                garraOut.closeClaw(),
                                new SleepAction(0.5),
                                garraIn.openClaw()


                        )
                )
                .endTrajectory();
        return tab1.build();
    }
}

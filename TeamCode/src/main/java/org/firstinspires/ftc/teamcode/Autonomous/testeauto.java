/*package org.firstinspires.ftc.teamcode.Autonomous;

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

import java.util.Arrays;
@Autonomous
public class testeauto extends LinearOpMode {
    @Override
    public void runOpMode() {


        waitForStart();
        Actions.runBlocking(
                BlueLeft()
        );

    }

    private Action BlueLeft(){
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
}
*/
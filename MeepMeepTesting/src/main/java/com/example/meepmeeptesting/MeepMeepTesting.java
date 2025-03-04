package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d initialPose = new Pose2d(38, 61, Math.toRadians(270));

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI/2)
        ));

        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-30, 50);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                //Deposito
                .splineTo(new Vector2d(52.89383390875461, 51.97265295027128), Math.toRadians(220), baseVelConstraint, baseAccelConstraint)
                //Primeiro sample
                .splineTo(new Vector2d( 48.464445832304484,43.2), Math.toRadians(270), null, baseAccelConstraint)
                //Deposito primeiro sample
                .strafeToLinearHeading(new Vector2d( 53.882067894113916, 50.47508719599699), Math.toRadians(230), baseVelConstraint, baseAccelConstraint)
                //2° Amostra do meio
                .strafeToLinearHeading(new Vector2d( 59.8569050492192, 43.842050718476024), Math.toRadians(270), baseVelConstraint, baseAccelConstraint)
                //Deposito segunda a mostra
                .strafeToLinearHeading(new Vector2d(52.89383390875461,  51.97265295027128), Math.toRadians(230), baseVelConstraint, baseAccelConstraint)
                .setTangent(Math.toRadians(230))
                .splineToLinearHeading(new Pose2d(25, 14, Math.toRadians(270)), Math.toRadians(180), null, new ProfileAccelConstraint(-250, 250))
                .endTrajectory()
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
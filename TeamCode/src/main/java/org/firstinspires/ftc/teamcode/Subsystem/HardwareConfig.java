package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class HardwareConfig {


    // Motores do Slide Outtake
    public DcMotorEx outtakeSlideL;
    public DcMotorEx outtakeSlideR;

    // Motores do Slide Intake
    public DcMotorEx intakeSlide;

    // Sensores de toque
    public TouchSensor tLeft;
    public TouchSensor tRight;

    // Servos Intake
    public Servo servoIntakeL;
    public Servo servoIntakeR;
    public Servo clawIn;

    // Servos Outtake
    public Servo servoOuttakeL;
    public Servo servoOuttakeR;
    public Servo clawOut;
    public DcMotor outtake;



    public HardwareConfig(HardwareMap hardwareMap) {


        // Inicializando motores e definindo direções
        outtakeSlideL = hardwareMap.get(DcMotorEx.class, "mSlideL");
        outtakeSlideR = hardwareMap.get(DcMotorEx.class, "mSlideR");

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        intakeSlide = hardwareMap.get(DcMotorEx.class, "IntakeSlide");

        // Inicializando servos
        servoIntakeL = hardwareMap.get(Servo.class, "sIntakeLeft");
        servoIntakeR = hardwareMap.get(Servo.class, "sIntakeRight");
        clawIn = hardwareMap.get(Servo.class, "ClawIn");
        clawOut = hardwareMap.get(Servo.class, "ClawOut");

        // Configurando direções dos servos
        servoIntakeL.setDirection(Servo.Direction.REVERSE);

        outtakeSlideL.setDirection(DcMotorSimple.Direction.REVERSE);


        //configura direção do outtake
        outtake.setDirection(DcMotorSimple.Direction.REVERSE
        );

        clawIn.setDirection(Servo.Direction.REVERSE);
        clawOut.setDirection(Servo.Direction.REVERSE);

        //Outtake Slide
        outtakeSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake Slide
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //outtake
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Inicializando sensores de toque
        tLeft = hardwareMap.get(TouchSensor.class, "tLeft");
        tRight = hardwareMap.get(TouchSensor.class, "tRight");
    }
}

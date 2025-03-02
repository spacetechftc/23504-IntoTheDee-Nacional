package org.firstinspires.ftc.teamcode.Anexos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp()
public class Diff extends OpMode{

    public Servo garraIn;
    public Servo dE;
    public Servo dD;
    double tetha1;
    double tetha2;
    double tethaP;
    double tethaR;


    @Override
    public void init(){
        garraIn = hardwareMap.get(Servo.class, "ClawIn");
        dE = hardwareMap.get(Servo.class, "sIntakeLeft");
        dD = hardwareMap.get(Servo.class, "sIntakeRight");
        dD.setDirection(Servo.Direction.FORWARD);
        dE.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop(){

        tethaR = gamepad1.right_stick_x;
        tethaP = -gamepad1.left_stick_y;

        tetha1 = tethaP + (tethaR / 2); //Angulo servo esquerda
        tetha2 = tethaP - (tethaR / 2); //Angulo servo direita

        tetha1 = ConvertRange(tetha1, -1, 1, 0, 1);
        tetha2 = ConvertRange(tetha2, -1, 1, 0, 1);

        dE.setPosition(tetha1);
        dD.setPosition(tetha2);

        if(gamepad1.b){
            garraIn.setPosition(1);
        } if(gamepad1.a){
            garraIn.setPosition(0);
        }


        telemetry.addData("servo L", dE.getPosition());
        telemetry.addData("servo D", dD.getPosition());


    }



    public double ConvertRange(double x, double in_min, double in_max, double out_min, double out_max){

        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    }
}
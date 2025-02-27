package org.firstinspires.ftc.teamcode.Anexos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;

@Disabled
@Config
public class Servos extends OpMode {
    Servo Sl;
    Servo Sr;
    DcMotor SlideIntake;
    public double Max = 1000;

    HardwareConfig hardware = new HardwareConfig(hardwareMap);
    //LiftControl liftControl = new LiftControl(hardware, ValoresSlide.p, ValoresSlide.i, ValoresSlide.d, ValoresSlide.f, 537.7 / 360);

    @Override
    public void init() {
        Sl = hardwareMap.get(Servo.class, "ServoOuttakel");
        Sr = hardwareMap.get(Servo.class, "ServoOuttaker");
        SlideIntake = hardwareMap.get(DcMotor.class, "SlideIntake");
        Sr.setDirection(Servo.Direction.REVERSE);
        Sr.setPosition(0.0);
        Sl.setPosition(0.0);

        SlideIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        double SlidePower = gamepad1.left_stick_y;
        //double targetAngle = map(SlideIntake.getCurrentPosition(), 0, liftControl.target, 0, 1);

        SlideIntake.setPower(SlidePower);
        //Sl.setPosition(targetAngle);
        //Sr.setPosition(targetAngle);

        telemetry.addData("Slide", getSlidePosition());
        //telemetry.addData("Sr e Sl", targetAngle);
        telemetry.update();

    }

    double getSlidePosition(){
        int armPos = SlideIntake.getCurrentPosition();
        double CPR = 288; // Mudar isso
        double revolutions = armPos/CPR;
        double angle = (revolutions* 360);
        return angle;
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}

package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Subsystem.Valores.ValoresPIDFIntake;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.ValoresPIDFOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.ValoresPIDFSlide;

@Disabled
@Config
public class Slides_Methods {

    public static double psOut = ValoresPIDFSlide.p, isOut = ValoresPIDFSlide.i, dsOut = ValoresPIDFSlide.d;
    public static double fsOut = ValoresPIDFSlide.f;

    public static double pOut = ValoresPIDFOuttake.p, iOut = ValoresPIDFOuttake.i, dOut = ValoresPIDFOuttake.d;
    public static double fOut = ValoresPIDFOuttake.f;

    public static double pIn = ValoresPIDFIntake.p, iIn = ValoresPIDFIntake.i, dIn = ValoresPIDFIntake.d;
    public static double fIn = ValoresPIDFIntake.f;

    public static final double ticks_in_degreeSlideOut = 537.6 / 180;
    public static final double ticks_in_degreeOut = 288.0 / 180;
    public static final double ticks_in_degreeInt = 765 * -0.3 / 180;


    public static double returnPIDSlideOut(double currentPos, double target) {
        PIDController controllerSOut = new PIDController(psOut, isOut, dsOut);
        controllerSOut.setPID(psOut, isOut, dsOut);
        double pid = controllerSOut.calculate(currentPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degreeSlideOut)) * fsOut;
        return pid + ff;
    }

    public static double returnPIDOut(double currentPos, double target) {
        PIDController controllerOut = new PIDController(pOut, iOut, dOut);
        controllerOut.setPID(pOut, iOut, dOut);
        double pid = controllerOut.calculate(currentPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degreeOut)) * fOut;

        return pid + ff;
    }

    public static double returnPIDIn(double currentPos, double target) {
        PIDController controllerIn = new PIDController(pIn, iIn, dIn);
        controllerIn.setPID(pIn, iIn, dIn);
        double pid = controllerIn.calculate(currentPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degreeInt)) * fIn;
        return pid + ff;


    }
}

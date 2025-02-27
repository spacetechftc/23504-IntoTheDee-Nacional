package org.firstinspires.ftc.teamcode.Anexos.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.ValoresPIDFSlide;

@Config
public class VSlidePIDF extends LinearOpMode {

    public static double p = ValoresPIDFSlide.p, i = ValoresPIDFSlide.i, d = ValoresPIDFSlide.d;
    public static double f = ValoresPIDFSlide.f;

    public static int target = 0;

    public final double ticks_in_degree = 537.6/180;

    @Override
    public void runOpMode() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        PIDController controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                controller.setPID(p, i, d);
                int armPos = hw.outtakeSlideL.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                double power = pid + ff;

                hw.outtakeSlideL.setPower(power);
                hw.outtakeSlideR.setPower(power);

                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);
                telemetry.addData("power ", power);

                telemetry.update();
            }
        }
    }
}
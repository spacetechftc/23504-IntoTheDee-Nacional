package org.firstinspires.ftc.teamcode.Anexos.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;


@Config
@Disabled
public class IntakePIDF extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.01, i = 0.0, d = 0.00001;
    public static double f = 0.01;

    public static int target = 0;

    public final double ticks_in_degree = 765 * -0.3 / 180;

    @Override
    public void runOpMode() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                controller.setPID(p, i, d);
                int armPos = hw.intakeSlide.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                double power = pid + ff;

                hw.intakeSlide.setPower(power);

                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);
                telemetry.addData("power ", power);

                telemetry.update();
            }
        }
    }
}
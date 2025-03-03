package org.firstinspires.ftc.teamcode.Anexos.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.ValoresPIDFOuttake;

@Config
@TeleOp
public class OuttakePIDF extends LinearOpMode {

    public static double p = ValoresPIDFOuttake.p , i = ValoresPIDFOuttake.i, d = ValoresPIDFOuttake.d;
    public static double f = ValoresPIDFOuttake.f;

    public static int target = 0;

    public final double ticks_in_degree = 533.0416/360;

    @Override
    public void runOpMode() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        PIDController controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hw.outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                controller.setPID(p, i, d);
                int armPos = hw.outtake.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                double power = pid + ff;


                hw.outtake.setPower(power);

                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);
                telemetry.addData("power ", power);

                telemetry.update();
            }
        }
    }
}
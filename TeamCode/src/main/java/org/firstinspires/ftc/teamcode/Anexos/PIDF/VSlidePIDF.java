package org.firstinspires.ftc.teamcode.Anexos.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.Slides_Methods;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.ValoresPIDFSlide;

@Disabled
@Config
public class VSlidePIDF extends LinearOpMode {

    public static double p = ValoresPIDFSlide.p, i = ValoresPIDFSlide.i, d = ValoresPIDFSlide.d;
    public static double f = ValoresPIDFSlide.f;

    public static int target = 0;

    public final double ticks_in_degree = 537.7 /360;
    double outcc = 0;

    @Override
    public void runOpMode() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        PIDController controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {

                if(gamepad1.a){
                    hw.outtakeSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hw.outtakeSlideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hw.outtakeSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hw.outtakeSlideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                if(gamepad1.x){
                    hw.outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hw.outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }

                controller.setPID(p, i, d);
                int armPos = hw.outtakeSlideL.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                controller.setSetPoint(target);
                controller.setTolerance(10);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                double power = pid + ff;

                hw.outtakeSlideL.setPower(power);
                hw.outtakeSlideR.setPower(power);


                if((controller.atSetPoint())){
                    if(target == 3250){
                        double outp = Slides_Methods.returnPIDOut(hw.outtake.getCurrentPosition(), 100);
                        hw.outtake.setPower(outp);
                    }
                }
                if(!(controller.atSetPoint())){
                    if(target == -1){
                        double outp = Slides_Methods.returnPIDOut(hw.outtake.getCurrentPosition(), 0);
                        hw.outtake.setPower(outp);
                    }
                }

                telemetry.addData("pos ", armPos);
                telemetry.addData("target ", target);
                telemetry.addData("power ", power);

                telemetry.update();
            }
        }
    }
}
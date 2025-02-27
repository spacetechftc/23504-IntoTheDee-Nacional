package org.firstinspires.ftc.teamcode.Anexos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;

public class Slide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            double power = gamepad1.left_stick_y;
            hw.outtake.setPower(power);

            telemetry.addData("Intake Slide position ", hw.outtake.getCurrentPosition());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.Anexos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;

public class ServoIn extends OpMode {

    private boolean isPressed = true;
    private boolean servoPosition = false;

    @Override
    public void init() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        hw.servoIntakeL.setPosition(Constants.INTAKE_EM_PE);
        hw.servoIntakeR.setPosition(Constants.INTAKE_EM_PE);
        hw.clawIn.setPosition(Constants.GARRA_INTAKE_ABERTA);
    }

    @Override
    public void loop() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        if (gamepad1.a) {
            hw.servoIntakeL.setPosition(Constants.INTAKE_EM_PE);
            hw.servoIntakeR.setPosition(Constants.INTAKE_EM_PE);
        } else if (gamepad1.b) {
            hw.servoIntakeL.setPosition(Constants.INTAKE_PRONTO_PARA_PEGAR);
            hw.servoIntakeR.setPosition(Constants.INTAKE_PRONTO_PARA_PEGAR);
        }

        if (gamepad1.x && !isPressed) {
            servoPosition = !servoPosition;
            if (servoPosition) {
                hw.clawIn.setPosition(Constants.GARRA_INTAKE_FECHADA);
            } else {
                hw.clawIn.setPosition(Constants.GARRA_INTAKE_ABERTA);
            }
        }
        isPressed = gamepad1.x;
    }
}

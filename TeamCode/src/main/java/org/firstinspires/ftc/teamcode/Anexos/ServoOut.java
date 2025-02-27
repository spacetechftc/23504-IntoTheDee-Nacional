package org.firstinspires.ftc.teamcode.Anexos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;

public class ServoOut extends OpMode {

    private boolean isPressed = true;
    private boolean servoPosition = false;

    @Override
    public void init() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_ABERTA);
        hw.servoOuttakeR.setPosition(Constants.OUTTAKE_PRONTO_PARA_PEGAR);
        hw.servoOuttakeL.setPosition(Constants.OUTTAKE_PRONTO_PARA_PEGAR);


    }

    @Override
    public void loop() {
        HardwareConfig hw = new HardwareConfig(hardwareMap);

        if (gamepad1.a) {
            hw.servoOuttakeR.setPosition(Constants.OUTTAKE_PRONTO_PARA_PEGAR);
            hw.servoOuttakeL.setPosition(Constants.OUTTAKE_PRONTO_PARA_PEGAR);
        } else if (gamepad1.b) {
            hw.servoOuttakeL.setPosition(Constants.OUTTAKE_PRONTO_PARA_JOGAR);
            hw.servoOuttakeR.setPosition(Constants.OUTTAKE_PRONTO_PARA_JOGAR);
        }

        if (gamepad1.x && hw.clawOut.getPosition() == Constants.GARRA_OUTTAKE_ABERTA) {
                hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_FECHADA);
            } else if (gamepad1.x && hw.clawOut.getPosition() == Constants.GARRA_OUTTAKE_FECHADA){
                hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_ABERTA);

        }
    }
}

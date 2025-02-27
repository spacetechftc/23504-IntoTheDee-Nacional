package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadHandler {
    // Botões do operador
    public final ButtonReader xButtonReader;
    public final ButtonReader bButtonReader;
    public final ButtonReader dpadUpReader;
    public final ButtonReader dpadDownReader;
    public final ButtonReader rightBumperReader;
    public final ButtonReader leftBumperReader;
    public final ButtonReader yButtonReader;
    public final ButtonReader aButtonReader;

    // Controle de movimentação do robô
    private final double forward;
    private final double strafe;
    private final double turn;

    private final double slideUpDown;


    public GamepadHandler(GamepadEx gamepad) {
        // Botões do operador
        xButtonReader = new ButtonReader(gamepad, GamepadKeys.Button.X);
        bButtonReader = new ButtonReader(gamepad, GamepadKeys.Button.B);
        dpadUpReader = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP);
        dpadDownReader = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN);
        rightBumperReader = new ButtonReader(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumperReader = new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER);
        yButtonReader = new ButtonReader(gamepad, GamepadKeys.Button.Y);
        aButtonReader = new ButtonReader(gamepad, GamepadKeys.Button.A);

        // Inicializando os controles de movimentação
        forward = gamepad.getLeftY();
        strafe = gamepad.getLeftX();
        turn = gamepad.getRightX();

        slideUpDown = gamepad.getRightY();
    }

    // Método para atualizar os botões e inputs de movimento
    public void update() {
        xButtonReader.readValue();
        bButtonReader.readValue();
        dpadUpReader.readValue();
        dpadDownReader.readValue();
        rightBumperReader.readValue();
        leftBumperReader.readValue();
        yButtonReader.readValue();
        aButtonReader.readValue();
    }

    public double getForward() {
        return forward;
    }

    public double getStrafe() {
        return strafe;
    }

    public double getTurn() {
        return turn;
    }

    public double upDown() {
        return slideUpDown;
    }
}
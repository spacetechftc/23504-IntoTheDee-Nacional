package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;

import java.util.List;
import java.util.ArrayList;



@TeleOp(name = "Sensor: Limelight ", group = "Control")
public class teste extends LinearOpMode {

    HardwareConfig hw = new HardwareConfig(hardwareMap);

    private Limelight3A limelight;
    private Servo S1;
    private int currentPipeline = -1; // Para evitar mudanças desnecessárias de pipeline
    private String orientacao;
    private int validDetectionCount = 0;
    double robotAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        S1 = hardwareMap.get(Servo.class, "S1");
        telemetry.setMsTransmissionInterval(100);

        limelight.pipelineSwitch(0); // Definir um pipeline inicial
        currentPipeline = 0;

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                validDetectionCount++;
                if (validDetectionCount >= 3) { // Requer 3 leituras válidas consecutivas
                    processColorResults(result);
                    validDetectionCount = 0; // Reseta após atualizar
                }
            } else {
                validDetectionCount = 0;
                telemetry.addData("Limelight", "No valid data");
            }
            telemetry.update();
            intakeControl();
        }
    }

    private void processColorResults(LLResult result) {
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

        for (LLResultTypes.ColorResult cr : colorResults) {
            if (cr.getTargetCorners() == null || cr.getTargetCorners().size() < 2) {
                telemetry.addData("Erro", "Não há cantos suficientes detectados!");
                continue;
            }

            double robotAngle = cr.getTargetXDegrees();
            double angle = calculateAngle(cr);
            orientacao = calculateOrientation(angle); // Determina a orientação com base no ângulo
            telemetry.addData("Ângulo", angle);
            telemetry.addData("Orientação", orientacao);
        }
    }

    public double angleTurn(LLResult result){
        limelight.pipelineSwitch(0);
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

        for (LLResultTypes.ColorResult cr : colorResults) {
            if (cr.getTargetCorners() == null || cr.getTargetCorners().size() < 2) {
                telemetry.addData("Erro", "Não há cantos suficientes detectados!");
                continue;
            }

            robotAngle = cr.getTargetXDegrees();

        }
        return robotAngle;

    }
    private String calculateOrientation(double angle) {
        // Normaliza o ângulo para um intervalo de 0 a 180 graus
        double normalizedAngle = Math.abs(angle) % 180;

        if (normalizedAngle < 60 || normalizedAngle > 155) {
            return "Horizontal";
        } else if (normalizedAngle > 65 && normalizedAngle < 115) {
            return "Vertical";
        }
        return "Indefinido"; // Caso não se encaixe nas faixas estabelecidas
    }


    private double[] calculateRawMoments(List<List<Double>> corners) {
        double m00 = 0, m10 = 0, m01 = 0, m20 = 0, m02 = 0, m11 = 0;

        for (List<Double> point : corners) {
            if (point.size() < 2) continue; // Evita erros

            double x = point.get(0);
            double y = point.get(1);

            m00 += 1;
            m10 += x;
            m01 += y;
            m20 += x * x;
            m02 += y * y;
            m11 += x * y;
        }

        if (m00 == 0) return new double[]{0, 0, 0}; // Evita divisão por zero

        double cx = m10 / m00;
        double cy = m01 / m00;
        double mu20 = m20 / m00 - cx * cx;
        double mu02 = m02 / m00 - cy * cy;
        double mu11 = m11 / m00 - cx * cy;

        return new double[]{mu20, mu02, mu11};
    }

    private double calculateAngle(LLResultTypes.ColorResult cr) {
        List<List<Double>> corners = cr.getTargetCorners();
        if (corners == null || corners.size() < 2) {
            return 0; // Proteção contra erros
        }

        double[] moments = calculateRawMoments(corners);
        double mu20 = moments[0];
        double mu02 = moments[1];
        double mu11 = moments[2];

        if (mu20 - mu02 == 0) {
            return 0; // Evita divisão por zero
        }

        double angle = 0.5 * Math.atan2(2 * mu11, mu20 - mu02);
        return Math.toDegrees(angle);
    }

    public void servoControl() {
        if (orientacao.equals("Horizontal")) {
            hw.servoIntakeL.setPosition(Constants.INTAKE_COLETA_HORIZONTAL_L);
            hw.servoIntakeR.setPosition(Constants.INTAKE_COLETA_HORIZOLTAL_D);
        } else if (orientacao.equals("Vertical")) {
            hw.servoIntakeL.setPosition(Constants.INTAKE_COLETAR_VERTICAL_L);
            hw.servoIntakeR.setPosition(Constants.INTAKE_COLETAR_VERTICAL_D);
        }
    }

    private void intakeControl() {
        if (gamepad1.b) {
            switchPipeline(1);
            servoControl();
        } else if (gamepad1.y) {
            switchPipeline(2);
            servoControl();
        } else if (gamepad1.x) {
            switchPipeline(0);
            servoControl();
        }
    }

    private void switchPipeline(int pipeline) {
        if (currentPipeline != pipeline) {
            limelight.pipelineSwitch(pipeline);
            currentPipeline = pipeline;
            telemetry.addData("Pipeline Switch", "Mudando para " + pipeline);
            sleep(100); // Pequena pausa para garantir a mudança de pipeline
        }
    }
}
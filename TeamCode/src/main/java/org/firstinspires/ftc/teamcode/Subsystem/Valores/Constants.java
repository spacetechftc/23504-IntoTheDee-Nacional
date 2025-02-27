package org.firstinspires.ftc.teamcode.Subsystem.Valores;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
                // Servos Intake
                public static  double INTAKE_EM_PE = 0.61;
                public static  double INTAKE_PRONTO_PARA_PEGAR = 0.0;

                // Servos Outtake
                public static  int OUTTAKE_PRONTO_PARA_PEGAR = -100;
                public static  int OUTTAKE_PRONTO_PARA_JOGAR = 100;

                // Posições do Slide em Ticks
                public static  int SLIDE_CESTA_ALTA = -4000;
                public static  int SLIDE_CESTA_BAIXA = -2000;
                public static  int SLIDE_BARRA_ESPECIME = -1500;

                // Garra Intake
                public static  double GARRA_INTAKE_ABERTA = 0.72;
                public static  double GARRA_INTAKE_FECHADA = 0.0;

                // Garra Outtake
                public static  double GARRA_OUTTAKE_ABERTA = 0.5;
                public static  double GARRA_OUTTAKE_FECHADA = 0.0;

}

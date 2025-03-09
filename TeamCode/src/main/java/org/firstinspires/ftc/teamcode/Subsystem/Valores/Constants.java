package org.firstinspires.ftc.teamcode.Subsystem.Valores;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
                // Servos Intake
                public static  double INTAKE_EM_PE = 0.61;
                public static  double INTAKE_PRONTO_PARA_PEGAR = 0.0;

                public static double INTAKE_COLETAR_VERTICAL_L = 1;
                public static double INTAKE_COLETAR_VERTICAL_D = 1;

                public static double INTAKE_PASSAGEM_L = 0.3796;
                public static double INTAKE_PASSAGEM_D = 0.3796;

                public static double INTAKE_RETRO_L = 0.1776;
                public static double INTAKE_RETRO_D = 0.1812;

                public static double INTAKE_COLETA_HORIZONTAL_L = 1;
                public static double INTAKE_COLETA_HORIZOLTAL_D = 1;


                // Servos Outtake
                public static  int OUTTAKE_PRONTO_PARA_PEGAR = -100;
                public static  int OUTTAKE_PRONTO_PARA_JOGAR = 100;

                // Posições do Slide em Ticks
                public static  int SLIDE_CESTA_ALTA = -4000;
                public static  int SLIDE_CESTA_BAIXA = -2000;
                public static  int SLIDE_BARRA_ESPECIME = -1500;

                // Garra Intake
                public static double GARRA_INTAKE_ABERTA = 1;
                public static double GARRA_INTAKE_FECHADA = 0;
                public static double GARRA_INTAKE_MED = 0.5;

                // Garra Outtake
                public static  double GARRA_OUTTAKE_ABERTA = 0;
                public static  double GARRA_OUTTAKE_FECHADA = 1;

}

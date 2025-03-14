package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Valores.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.HardwareConfig;
import org.firstinspires.ftc.teamcode.Subsystem.MathUtils;
import org.firstinspires.ftc.teamcode.Subsystem.Slides_Methods;

public class RobotActions {

    public static class Lift {
        private final HardwareConfig hw;
        private final Slides_Methods slidesMethods;

        public Lift(HardwareConfig hardwareConfig) {
            this.hw = hardwareConfig;
            this.slidesMethods = new Slides_Methods();
        }

        public class LiftToTarget implements Action {
            private boolean initialized = false;
                                                    private final int target;
            ElapsedTime timer = new ElapsedTime();

            public LiftToTarget(int target) {
                this.target = target;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                }

                double currentPositionLeft = hw.outtakeSlideL.getCurrentPosition();
                double error = target - currentPositionLeft;


                double power = slidesMethods.returnPIDSlideOut(currentPositionLeft, target);
                hw.outtakeSlideL.setPower(power);
                hw.outtakeSlideR.setPower(power);


                if((target == 3000 || target == 1130 || target == 1275) && error <10){
                    hw.outtakeSlideL.setPower(0.1);
                    hw.outtakeSlideR.setPower(0.1);
                    return false;
                } else if (power > 0) {
                    return true;// Valor fixo de descida
                }




                hw.outtake.setPower(0);


                // Verificação de interrupção por sensores
                if (hw.tLeft.isPressed() || hw.tRight.isPressed()) {
                    hw.outtakeSlideL.setPower(0);
                    hw.outtakeSlideR.setPower(0);
                    return false;
                }

                if(timer.seconds() > 3.5){
                    if ((target == 3000 || target == 1130) || (target == 1275)) {
                        hw.outtakeSlideL.setPower(0.1);
                        hw.outtakeSlideR.setPower(0.1);
                    } else if( target== 0 ){
                        hw.outtakeSlideL.setPower(0);
                        hw.outtakeSlideR.setPower(0);
                    }
                    return false;
                }
                // Verifica se o erro está dentro da tolerância
                if (Math.abs(error) <= 10) {
                    if ((target == 3000 || target == 1130) || (target == 1275)) {
                        hw.outtakeSlideL.setPower(0.1);
                        hw.outtakeSlideR.setPower(0.1);
                    } else if (target == 0){
                        hw.outtakeSlideL.setPower(0);
                        hw.outtakeSlideR.setPower(0);

                    }
                    return false;
                } else{
                    return true;
                }





            }


        }

        public Action liftToTarget(int target) {
            return new LiftToTarget(target);
        }
    }


    public static class ExtensionControl{
        private final HardwareConfig hw;
        private final Slides_Methods slidesMethods;
        ElapsedTime timer = new ElapsedTime();


        public ExtensionControl(HardwareConfig hardwareConfig) {
            this.hw = hardwareConfig;
            this.slidesMethods = new Slides_Methods();
        }

        public class ExtendToTarget implements Action {

            private boolean initialized = false;
            private final int target;

            public ExtendToTarget(int target) {
                this.target = target;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                double currentPosition = hw.intakeSlide.getCurrentPosition();
                double power = slidesMethods.returnPIDIn(currentPosition, target);


                if(target > 0){
                    hw.intakeSlide.setPower(1);
                } else {
                    hw.intakeSlide.setPower(-1);
                }

                if (timer.seconds() > 1.5) {
                    hw.intakeSlide.setPower(0);
                    return false;
                }

                return true;
            }
        }

        public Action extendTarget(int target) {
            return new ExtendToTarget(target);
        }

        public class ExtendTLit implements Action {

            private boolean initialized = false;
            private final int target;

            public ExtendTLit(int target) {
                this.target = target;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                double currentPosition = hw.intakeSlide.getCurrentPosition();
                double power = slidesMethods.returnPIDIn(currentPosition, target);


                if(target > 0){
                    hw.intakeSlide.setPower(power);
                } else {
                    hw.intakeSlide.setPower(power);
                }

                if (timer.seconds() > 0.7) {
                    hw.intakeSlide.setPower(0);
                    return false;
                }

                return true;
            }
        }

        public Action extendTLit(int target) {
            return new ExtendTLit(target);
        }
    }

    public static class Intake{
        private final HardwareConfig hw;

        public Intake(HardwareConfig hardwareConfig){this.hw = hardwareConfig;}

        public class IntakePP implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                hw.servoIntakeL.setPosition(Constants.INTAKE_COLETAR_VERTICAL_L);
                hw.servoIntakeR.setPosition(Constants. INTAKE_COLETAR_VERTICAL_D);
                packet.put("servoIntakeL Position", hw.servoIntakeL.getPosition());
                packet.put("servoIntakeR Position", hw.servoIntakeR.getPosition());
                if(hw.servoIntakeR.getPosition() == Constants.INTAKE_COLETAR_VERTICAL_D
                        && hw.servoIntakeL.getPosition() == Constants.INTAKE_COLETAR_VERTICAL_L ) {
                return false;
                } else {
                    return true;
                }
            }
        }

        public Action VertColet(){ return  new Intake.IntakePP();}

        public class IntakeRR implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                hw.servoIntakeL.setPosition(Constants.INTAKE_RETRO_L);
                hw.servoIntakeR.setPosition(Constants.INTAKE_RETRO_D);
                packet.put("servoIntakeL Position", hw.servoIntakeL.getPosition());
                packet.put("servoIntakeR Position", hw.servoIntakeR.getPosition());
                return false;
            }
        }
        public Action retract(){ return  new Intake.IntakeRR();}

        public class IntakePD implements  Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                hw.servoIntakeL.setPosition(Constants.INTAKE_PASSAGEM_L);
                hw.servoIntakeR.setPosition(Constants.INTAKE_PASSAGEM_D);
                packet.put("servoIntakeL Position", hw.servoIntakeL.getPosition());
                packet.put("servoIntakeR Position", hw.servoIntakeR.getPosition());
                return false;
            }
        }
        public Action pass(){ return new Intake.IntakePD();}

        public class IntakeHC implements  Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                hw.servoIntakeL.setPosition(Constants.INTAKE_COLETA_HORIZONTAL_L);
                hw.servoIntakeR.setPosition(Constants.INTAKE_COLETA_HORIZOLTAL_D);
                packet.put("servoIntakeL Position", hw.servoIntakeL.getPosition());
                packet.put("servoIntakeR Position", hw.servoIntakeR.getPosition());
                return false;
            }
        }
        public Action HorColet(){ return new Intake.IntakeHC();}
    }

    public static class Outtake{
        private final HardwareConfig hw;
        private final Slides_Methods slidesMethods;
        ElapsedTime timerout = new ElapsedTime();


        public Outtake(HardwareConfig hardwareConfig) {
            this.hw = hardwareConfig;
            this.slidesMethods = new Slides_Methods();
        }

        public class OuttakeToTarget implements Action {

            private boolean initialized = false;
            private final int target;

            public OuttakeToTarget(int target) {
                this.target = target;
            }



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timerout.reset();
                }

                double outpower = 0.85 * target;
                hw.outtake.setPower(outpower);



                if (timerout.seconds() >1.5) {
                    hw.outtake.setPower(0);
                    return false;
                } else {
                    return true;

                }
            }
        }

        public Action outtakeToTarget(int target) {
            return new OuttakeToTarget(target);
        }
    }

    public static class ClawIn{
        private final HardwareConfig hw;

        public ClawIn(HardwareConfig hardwareConfig){this.hw = hardwareConfig; }

        public class CloseClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                hw.clawIn.setPosition((Constants.GARRA_INTAKE_FECHADA));
                return false;
            }
        }

        public Action closeClaw() {
            return new ClawIn.CloseClaw();
        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                hw.clawIn.setPosition(Constants.GARRA_INTAKE_ABERTA);
                return false;
            }

        }

        public Action openClaw() {return new ClawIn.OpenClaw();}


        public class MidClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                hw.clawIn.setPosition(Constants.GARRA_INTAKE_MED);
                return false;
            }
        }

        public Action midClaw() {return new ClawIn.MidClaw();}


    }

    public static class ClawOut {
        private final HardwareConfig hw;

        public ClawOut(HardwareConfig hardwareConfig) {
            this.hw = hardwareConfig;
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_FECHADA);
                return false;
            }
        }

        public Action closeClaw() {
            return new ClawOut.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hw.clawOut.setPosition(Constants.GARRA_OUTTAKE_ABERTA);
                return false;
            }
        }

        public Action openClaw() {
            return new ClawOut.OpenClaw();
        }
    }



}

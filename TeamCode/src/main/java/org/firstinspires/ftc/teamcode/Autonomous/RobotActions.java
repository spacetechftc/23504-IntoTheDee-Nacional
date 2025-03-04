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
                    if(target != 0) {
                        hw.outtakeSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        hw.outtakeSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        hw.outtakeSlideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hw.outtakeSlideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    timer.reset();
                }

                double currentPositionLeft = hw.outtakeSlideL.getCurrentPosition();
                double error = currentPositionLeft - target;


                double power = slidesMethods.returnPIDSlideOut(currentPositionLeft, target);
                hw.outtakeSlideL.setPower(power);
                hw.outtakeSlideR.setPower(power);

                double outtakePower;

                if(target == 3200){
                    if(Slides_Methods.SlideOutIsAtSetpoint() || currentPositionLeft >= target - 100){

                        hw.outtakeSlideL.setPower(0);
                        hw.outtakeSlideR.setPower(0);
                        return false;

                    } else if (power > 0) {
                        return true;// Valor fixo de descida
                    }
                }

                if(Slides_Methods.SlideOutIsAtSetpoint()){

                    hw.outtakeSlideL.setPower(0);
                    hw.outtakeSlideR.setPower(0);
                    return false;

                } else if (power > 0) {
                    return true;// Valor fixo de descida
                }

                hw.outtake.setPower(0);



                if(target == 0) {
                    // Verificação de interrupção por sensores
                    if (hw.tLeft.isPressed() || hw.tRight.isPressed()) {
                        hw.outtakeSlideL.setPower(0);
                        hw.outtakeSlideR.setPower(0);
                        return false;
                    }
                }

                if(timer.seconds() > 3.5){
                    hw.outtakeSlideL.setPower(0);
                    hw.outtakeSlideR.setPower(0);
                    hw.outtake.setPower(0);
                    return false;
                }

                else{
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

                if (timer.seconds() > 3) {
                    hw.intakeSlide.setPower(0);
                    return false;
                }

                return true;
            }
        }

        public Action extendTarget(int target) {
            return new ExtendToTarget(target);
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
                return false;
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


                if (target > 1){
                    hw.outtake.setPower(1);
                } else {
                    hw.outtake.setPower(-1);
                }


                if (timerout.seconds() >1) {
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

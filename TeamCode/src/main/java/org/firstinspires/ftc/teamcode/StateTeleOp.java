package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "StateTeleOp")
public class StateTeleOp extends LinearOpMode {
    private RevBlinkinLedDriver blinkin;

    private DcMotor RightFront;
    private DcMotor RightBack;


    private DcMotor LeftFront;
    private DcMotor LeftBack;
//Mechanisms
    public class Intake {
        private Servo rightext;
        private Servo leftext;
        private Servo inwrist;
        private Servo inclaw;
        private Servo inarm;
        private Servo inbelt;
        double inwristpos;
        double inarmdownpos;

        public Intake(HardwareMap hardwareMap){
            rightext = hardwareMap.get(Servo.class, "rightext");
            inarm = hardwareMap.get(Servo.class, "inarm");
            inbelt = hardwareMap.get(Servo.class, "inbelt");
            inclaw = hardwareMap.get(Servo.class, "inclaw");
            inwrist = hardwareMap.get(Servo.class, "inwrist");
            leftext = hardwareMap.get(Servo.class, "leftext");
            rightext.setDirection(Servo.Direction.REVERSE);
            inwristpos = 0.5;
            inarmdownpos = 0.7;
        }
        //Actions
    public class HorizontalFullExtension implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rightext.setPosition(0.45);
            leftext.setPosition(0.39);
            return false;
        }
    }
    public Action horizontalfullextension(){
        return new HorizontalFullExtension();

    }public class HorizontalRetraction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rightext.setPosition(0.36);
            leftext.setPosition(0.3);
            return false;
        }
    }
    public Action horizontalretraction(){
        return new HorizontalRetraction();
    }

    public class HorizontalHalfExtension implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rightext.setPosition(0.39);
            leftext.setPosition(0.33);
            return false;
        }
    }
    public Action horizontalhalfextension(){
        return new HorizontalHalfExtension();
    }

    public class InArmDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inarm.setPosition(inarmdownpos);
            inbelt.setPosition(0.83);
            return false;
        }
    }
    public Action inarmdown(){
        return new InArmDown();
    }
    public class InArmUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inarm.setPosition(inarmdownpos - 0.045);
            inbelt.setPosition(0.9);
            return false;
        }
    }
    public Action inarmup(){
        return new InArmUp();
    }
    public class InArmBack implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inarm.setPosition(inarmdownpos - 0.179);
            inbelt.setPosition(0.17);
            return false;
        }
    }
    public Action inarmback(){
        return new InArmBack();
    }
    public class InClawExtend implements Action{
            @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                inclaw.setPosition(0.8);
                return false;
            }
    }
    public Action inclawextend(){
            return new InClawExtend();
    }
    public class InClawRetract implements Action{
            @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                inclaw.setPosition(0.15);
                return false;
            }

    }
    public Action inclawretract(){
            return new InClawRetract();
    }
    public class InWristLeft implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (inwrist.getPosition() > 0){
                inwrist.setPosition(inwristpos - 0.1);
                sleep(75);
                inwristpos = inwristpos - 0.1;
            }
            return false;
        }

    }
    public Action inwristleft(){
        return new InWristLeft();
    }
    public class InWristRight implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (inwrist.getPosition() < 1){
                inwrist.setPosition(inwristpos + 0.1);
                sleep(75);
                inwristpos = inwristpos + 0.1;
            }
            return false;
        }

    }
    public Action inwristright(){
        return new InWristRight();
    }
    public class InWristReset implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inwrist.setPosition(0.5);
            sleep(75);
            inwristpos = 0.5;
            return false;
        }

    }
    public Action inwristreset(){
        return new InWristReset();
    }

    public class InWristZero implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inwrist.setPosition(0);
            return false;
        }

    }
    public Action inwristzero(){
        return new InWristZero();
    }

    public class Transfer implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inarm.setPosition(0.6);
            inbelt.setPosition(0.5);


            return false;
        }

    }
    public Action transfer(){
        return new Transfer();
    }

    }



    public class Outtake{
        private Servo outarm;
        private Servo outbelt;
        private Servo outwrist;
        private Servo outclaw;
         double outarmtransferpos;

        public Outtake(HardwareMap hardwareMap){
            outarm = hardwareMap.get(Servo.class, "outarm");
            outbelt = hardwareMap.get(Servo.class, "outbelt");
            outwrist = hardwareMap.get(Servo.class, "outwrist");
            outclaw = hardwareMap.get(Servo.class, "outclaw");
            outarmtransferpos = 0.177;

        }

        public class OutWristReset implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outwrist.setPosition(0.51);
                return false;
            }

        }
        public Action outwristreset(){
            return new OutWristReset();
        }

        public class OutWristLeft implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outwrist.setPosition(0.17);
                return false;
            }

        }
        public Action outwristleft(){
            return new OutWristLeft();
        }
        public class OutWristRight implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outwrist.setPosition(0.85);
                return false;
            }

        }
        public Action outwristright(){
            return new OutWristRight();
        }


        public class OutClawExtend implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outclaw.setPosition(0.78);
                return false;
            }

        }
        public Action outclawextend(){
            return new OutClawExtend();
        }

        public class OutClawRetract implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outclaw.setPosition(0.12);
                return false;
            }

        }
        public Action outclawretract(){
            return new OutClawRetract();
        }

        public class OutArmDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outarm.setPosition(outarmtransferpos);
                outbelt.setPosition(0.565);
                return false;
            }

        }
        public Action outarmdown(){
            return new OutArmDown();
        }

        public class OutArmUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outarm.setPosition(1);
                outbelt.setPosition(0.46);
                return false;
            }

        }
        public Action outarmup(){
            return new OutArmUp();
        }

    }


    public class Lift{
        private DcMotor Leftlift;
        private DcMotor Rightlift;
        int lastliftpos;
        int liftlistnum;
        List<Integer> liftposes = new ArrayList<>();
        public Lift(HardwareMap hardwareMap){
            Leftlift = hardwareMap.get(DcMotor.class, "Leftlift");
            Rightlift = hardwareMap.get(DcMotor.class, "Rightlift");
            Leftlift.setDirection(DcMotor.Direction.REVERSE);
            Rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftlistnum = 2;
            liftposes.add(0);
            liftposes.add(500);
            liftposes.add(750);
            liftposes.add(2250);
            lastliftpos = liftposes.get(liftlistnum);

        }

        public class LiftUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(gamepad2.dpad_up){ //&& liftlistnum > 0 && liftlistnum < 3) {
                    lastliftpos = 2200; //liftposes.get(liftlistnum + 1);

                    Leftlift.setTargetPosition(lastliftpos);
                    Rightlift.setTargetPosition(lastliftpos);
                    Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Leftlift.setPower(0.7);
                    Rightlift.setPower(0.7);

                }

                return false;
            }

        }
        public Action liftup(){
            return new LiftUp();
        }

        public class LiftDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                    lastliftpos = 0;

                    Leftlift.setTargetPosition(lastliftpos);
                    Rightlift.setTargetPosition(lastliftpos);
                    Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Leftlift.setPower(0.8);
                    Rightlift.setPower(0.8);



                return false;
            }

        }
        public Action liftdown(){
            return new LiftDown();
        }


        public class LiftReset implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                Leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return false;
            }

        }
        public Action liftreset(){
            return new LiftReset();
        }




    }







    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double inwristpos;
        int lastliftpos;
        int liftamount;
        float forwardpos;
        float horizontalpos;
        float headingpos;
        boolean liftdown;
        boolean outclawext;
        boolean intakein;
        double drivespeed;
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");


        // Put initialization blocks here.
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);

        inwristpos = 0.5;
        lastliftpos = 2000;
        liftamount = 250;
        outclawext = false;
        liftdown = true;
        intakein = true;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            // set outtake arm on tele start
            Actions.runBlocking(
                    new SequentialAction(
                        outtake.outarmdown(),
                            outtake.outwristreset(),
                            outtake.outclawretract(),
                            lift.liftreset()
                    )
            );

            while (opModeIsActive()) {
                // Put loop blocks here.
                // driving blocks
                forwardpos = gamepad1.left_stick_y;
                horizontalpos = -gamepad1.left_stick_x;
                headingpos = -gamepad1.right_stick_x;

                if (intakein){
                    drivespeed = 0.6;
                } else{
                    drivespeed = 0.4;
                }



                RightFront.setPower((-headingpos + (forwardpos - horizontalpos)) * drivespeed);
                RightBack.setPower((-headingpos + forwardpos + horizontalpos) * drivespeed);
                LeftFront.setPower((headingpos + forwardpos + horizontalpos) * drivespeed);
                LeftBack.setPower((headingpos + (forwardpos - horizontalpos)) * drivespeed);
                // intake arm grab position
                if (gamepad1.x) {
                    Actions.runBlocking(intake.inarmup());
                }
                if (gamepad1.y) {
                    Actions.runBlocking(intake.inarmdown());
                }
                // intake claw blocks
                if (gamepad1.a) {
                    Actions.runBlocking(intake.inclawretract());
                }
                if (gamepad1.b) {
                    Actions.runBlocking(intake.inclawextend());
                }
                // rotate intake wirst
                if (gamepad1.left_bumper) {
                    Actions.runBlocking(intake.inwristleft());
                }
                if (gamepad1.right_bumper) {
                    Actions.runBlocking(intake.inwristright());
                }
                if (gamepad1.start) {
                    Actions.runBlocking(intake.inwristreset());
                }
                // return intake for transfer
                if (gamepad1.back) {
                    intakein = true;
                    Actions.runBlocking(
                            new SequentialAction(
                                    intake.inarmback(),
                                    intake.inwristzero(),
                                    intake.horizontalretraction()
                            )
                    );

                }
                // extend/ retract intake
                if (gamepad1.right_trigger >= 0.5) {
                    intakein = false;
                    Actions.runBlocking(intake.horizontalfullextension());
                }
                if (gamepad1.left_trigger >= 0.5) {
                    intakein = true;
                    Actions.runBlocking(intake.horizontalretraction());

                }

                // raise and lower lift
                if (gamepad2.dpad_up) {
                    Actions.runBlocking(
                      new SequentialAction(
                              lift.liftup(),
                              outtake.outarmup()
                      )
                    );
                }

                if (gamepad2.dpad_down) {
                    Actions.runBlocking(lift.liftdown());
                }
                // return lift
                if (gamepad2.back) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    lift.liftdown(),
                                    outtake.outarmdown(),
                                    outtake.outclawretract(),
                                    outtake.outwristreset()
                            )
                    );
                }
                // open/ close outtake claw
                if (gamepad2.x && liftdown) {
                    Actions.runBlocking(new SequentialAction(outtake.outclawextend()));
                    sleep(75);
                    Actions.runBlocking(new SequentialAction(
                            intake.inclawretract()));
                            sleep(200);
                            Actions.runBlocking(new SequentialAction(
                                    intake.horizontalhalfextension()
                            ));

                }
                if (gamepad2.y) {
                    Actions.runBlocking(outtake.outclawretract());
                }
                // rotate outtake wrist
                if (gamepad2.start) {
                    Actions.runBlocking(outtake.outwristreset());
                }
                if (gamepad2.dpad_right) {
                    Actions.runBlocking(outtake.outwristright());
                }
                if (gamepad2.dpad_left) {
                    Actions.runBlocking(outtake.outwristleft());
                }


                telemetry.update();
            }
        }
    }
}

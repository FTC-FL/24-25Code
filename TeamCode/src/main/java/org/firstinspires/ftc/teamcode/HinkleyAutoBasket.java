package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "HinkleyAutoBasket", preselectTeleOp = "DaytonTeleOpBasketSide")
public class HinkleyAutoBasket extends LinearOpMode {

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
            rightext.setPosition(0.39);
            leftext.setPosition(0.39);
            return false;
        }
    }
    public Action horizontalfullextension(){
        return new HorizontalFullExtension();

    }public class HorizontalRetraction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rightext.setPosition(0.295);
            leftext.setPosition(0.295);
            return false;
        }
    }
    public Action horizontalretraction(){
        return new HorizontalRetraction();
    }

    public class HorizontalHalfExtension implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rightext.setPosition(0.33);
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
            inarm.setPosition(inarmdownpos - 0.035);
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
            inarm.setPosition(inarmdownpos - 0.185);
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

                inwrist.setPosition(0);


            return false;
        }

    }
    public Action inwristleft(){
        return new InWristLeft();
    }
    public class InWristRight implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

                inwrist.setPosition(1);


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
            inarm.setPosition(inarmdownpos - 0.085);
            inbelt.setPosition(0.5);
            return false;
        }

    }
    public Action transfer(){
        return new Transfer();
    }
        public class Armstart implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inbelt.setPosition(0.5);
                inarm.setPosition(0.53);

                return false;
            }

        }
        public Action armstart(){
            return new Armstart();
        }
    }






    public class Outtake{
        private Servo outarm;
        private Servo outbelt;
        private Servo outwrist;
        private Servo outclaw;
        public Outtake(HardwareMap hardwareMap){
            outarm = hardwareMap.get(Servo.class, "outarm");
            outbelt = hardwareMap.get(Servo.class, "outbelt");
            outwrist = hardwareMap.get(Servo.class, "outwrist");
            outclaw = hardwareMap.get(Servo.class, "outclaw");
        }

        public class OutWristReset implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outwrist.setPosition(0.52);
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
                outarm.setPosition(0.785);
                outbelt.setPosition(0.56);
                return false;
            }

        }
        public Action outarmdown(){
            return new OutArmDown();
        }
        public class OutArmDownStart implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outarm.setPosition(0.93);
                outbelt.setPosition(0.489);
                return false;
            }

        }
        public Action outarmdownstart(){
            return new OutArmDownStart();
        }
        public class OutArmUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outarm.setPosition(0.62);
                outbelt.setPosition(0.48);
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

        public class LiftUpBasket implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                    lastliftpos = 3000;

                    Leftlift.setTargetPosition(lastliftpos);
                    Rightlift.setTargetPosition(lastliftpos);
                    Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Leftlift.setPower(1);
                    Rightlift.setPower(1);
                    sleep(100);



                return false;
            }

        }
        public Action liftupbasket(){
            return new LiftUpBasket();
        }
        public class LiftUpClip implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                lastliftpos = 600;

                Leftlift.setTargetPosition(lastliftpos);
                Rightlift.setTargetPosition(lastliftpos);
                Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Leftlift.setPower(0.5);
                Rightlift.setPower(0.5);
                sleep(100);



                return false;
            }

        }
        public Action liftupclip(){
            return new LiftUpClip();
        }
        public class LiftDownClip implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                lastliftpos = 0;
                sleep(1000);
                Leftlift.setTargetPosition(lastliftpos);
                Rightlift.setTargetPosition(lastliftpos);
                Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Leftlift.setPower(0.5);
                Rightlift.setPower(0.5);
                sleep(100);



                return false;
            }

        }
        public Action liftdownclip(){
            return new LiftDownClip();
        }
        public class LiftDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(gamepad2.dpad_down && liftlistnum > 0) {
                    lastliftpos = liftposes.get(liftlistnum - 1);

                    Leftlift.setTargetPosition(lastliftpos);
                    Rightlift.setTargetPosition(lastliftpos);
                    Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Leftlift.setPower(0.5);
                    Rightlift.setPower(0.5);
                    sleep(100);
                    liftlistnum = liftlistnum - 1;
                }
                else if(gamepad2.back && liftlistnum > 0){
                    Leftlift.setTargetPosition(liftposes.get(0));
                    Rightlift.setTargetPosition(liftposes.get(0));
                    Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Leftlift.setPower(0.5);
                    Rightlift.setPower(0.5);
                }
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

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(35, 61.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        Action basket1;
        Action basket1depo;
        Action getblock2;
        Action basket2;
        Action basket2depo;
        Action getblock3;
        Action basket3;


         basket1 = drive.actionBuilder(initialPose)
                        .strafeToLinearHeading(new Vector2d(54,51), Math.toRadians(225))
                                .build();
         basket1depo = drive.actionBuilder(new Pose2d(54,51,Math.toRadians(225)))
                 .strafeTo(new Vector2d(57,53))
                         .build();
         getblock2 = drive.actionBuilder(new Pose2d(57,53,Math.toRadians(225)))
                 .strafeToLinearHeading(new Vector2d(47,49),Math.toRadians(269))
                         .build();
        basket2 = drive.actionBuilder(new Pose2d(47, 49, Math.toRadians(269)))
                .strafeToLinearHeading(new Vector2d(53,50), Math.toRadians(225))
                .build();
        basket2depo = drive.actionBuilder(new Pose2d(53,50,Math.toRadians(225)))
                .strafeTo(new Vector2d(55,51))
                .build();
        getblock3 = drive.actionBuilder(new Pose2d(55, 51, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(56.5,49), Math.toRadians(259))
                        .build();
        basket3 = drive.actionBuilder(new Pose2d(56.5,49,Math.toRadians(259)))
                .strafeToLinearHeading(new Vector2d(53,50),Math.toRadians(245))
                .build();


        //init actions
        Actions.runBlocking(
                new SequentialAction(
                        intake.armstart(),
                        intake.inclawretract(),
                        outtake.outarmdownstart(),
                        outtake.outclawextend(),
                        intake.horizontalretraction(),
                        intake.inwristzero()
                )
        );
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            outtake.outwristreset(),
                            basket1,
                            lift.liftupbasket(),
                            outtake.outarmup()

                    )
            );
            Actions.runBlocking(new SleepAction(2));
            Actions.runBlocking(new SequentialAction(
                    basket1depo,
                    outtake.outclawretract(),
                    getblock2,
                    lift.liftdownclip(),
                    outtake.outarmdown(),
                    intake.horizontalfullextension(),
                    intake.inarmup()
            ));
            Actions.runBlocking(new SleepAction(1.5));
            Actions.runBlocking(new SequentialAction(intake.inarmdown()));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(intake.inclawextend()));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(intake.inarmback(),
                    intake.horizontalretraction()));
            Actions.runBlocking(new SleepAction(1.5));
            Actions.runBlocking(new SequentialAction(outtake.outclawextend()));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inclawretract()));
            Actions.runBlocking(new SleepAction(0.05));
            Actions.runBlocking(new SequentialAction(
                    intake.horizontalhalfextension()));
            Actions.runBlocking(new SleepAction(0.2));
            Actions.runBlocking(new SequentialAction(
                    intake.transfer(),
                    basket2,
                    lift.liftupbasket(),
                    outtake.outarmup()
                    ));
            Actions.runBlocking(new SleepAction(2));
            Actions.runBlocking(new SequentialAction(
                    basket2depo,
                    outtake.outclawretract()));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    getblock3,
                    lift.liftdownclip(),
                    outtake.outarmdown(),
                    intake.horizontalfullextension(),
                    intake.inarmup()
            ));
            Actions.runBlocking(new SleepAction(1.5));
            Actions.runBlocking(new SequentialAction(intake.inarmdown()));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(intake.inclawextend()));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(intake.inarmback(),
                    intake.horizontalretraction()));
            Actions.runBlocking(new SleepAction(1.5));
            Actions.runBlocking(new SequentialAction(outtake.outclawextend()));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inclawretract()));
            Actions.runBlocking(new SleepAction(0.05));
            Actions.runBlocking(new SequentialAction(
                    intake.horizontalhalfextension()));
            Actions.runBlocking(new SleepAction(0.2));
            Actions.runBlocking(new SequentialAction(
                    intake.transfer(),
                    basket3,
                    lift.liftupbasket(),
                    outtake.outarmup()
            ));

sleep(30000);
















































        }
    }
}

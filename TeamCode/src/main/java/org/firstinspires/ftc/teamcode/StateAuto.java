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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "StateAuto", preselectTeleOp = "StateTeleOp")
public class StateAuto extends LinearOpMode {

//Mechanisms

    private RevBlinkinLedDriver blinkin;


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

            inarmdownpos = 0.675;
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
            inbelt.setPosition(0.88);
            return false;
        }
    }
    public Action inarmdown(){
        return new InArmDown();
    }
    public class InArmUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inarm.setPosition(inarmdownpos - 0.02);
            inbelt.setPosition(0.93);
            return false;
        }
    }
    public Action inarmup(){
        return new InArmUp();
    }
    public class InArmBack implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inarm.setPosition(inarmdownpos - 0.165);
            inbelt.setPosition(0.25);
            return false;
        }
    }
    public Action inarmback(){
        return new InArmBack();
    }
    public class InClawExtend implements Action{
            @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                inclaw.setPosition(0.9);
                return false;
            }
    }
    public Action inclawextend(){
            return new InClawExtend();
    }
    public class InClawRetract implements Action{
            @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                inclaw.setPosition(0.25);
                return false;
            }

    }
    public Action inclawretract(){
            return new InClawRetract();
    }
    public class InWristLeft implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

                inwrist.setPosition(0.03);


            return false;
        }

    }
    public Action inwristleft(){
        return new InWristLeft();
    }
    public class InWristRight implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

                inwrist.setPosition(0.71);


            return false;
        }

    }
    public Action inwristright(){
        return new InWristRight();
    }
    public class InWristReset implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inwristpos = 0.37;
            inwrist.setPosition(inwristpos);
            return false;
        }

    }
    public Action inwristreset(){
        return new InWristReset();
    }

    public class InWristZero implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inwristpos = 0.03;
            inwrist.setPosition(inwristpos);
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
                inarm.setPosition(0.55);
                inbelt.setPosition(0.5);

                return false;
            }

        }
        public Action armstart(){
            return new Armstart();
        }

    public class WristMove implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            inwrist.setPosition(0.6);

            return false;
        }

    }
    public Action wristmove(){
        return new WristMove();
    }
}






    public class Outtake{
        private Servo outarm;
        private Servo outbelt;

        private Servo outclaw;
        double outarmtransferpos;
        public Outtake(HardwareMap hardwareMap){
            outarm = hardwareMap.get(Servo.class, "outarm");
            outbelt = hardwareMap.get(Servo.class, "outbelt");
            outclaw = hardwareMap.get(Servo.class, "outclaw");
            outarmtransferpos = 0.53;
        }




        public class OutClawExtend implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outclaw.setPosition(0.15);
                return false;
            }

        }
        public Action outclawextend(){
            return new OutClawExtend();
        }

        public class OutClawRetract implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outclaw.setPosition(0.45);
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
                outbelt.setPosition(0.455);
                return false;
            }

        }
        public Action outarmdown(){
            return new OutArmDown();
        }

        public class OutArmUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outarm.setPosition(0.82);
                outbelt.setPosition(0.425);
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
        public DcMotor LeftHang;
        public DcMotor RightHang;
        int lastliftpos;
        int liftlistnum;
        List<Integer> liftposes = new ArrayList<>();
        public Lift(HardwareMap hardwareMap){
            Leftlift = hardwareMap.get(DcMotor.class, "Leftlift");
            Rightlift = hardwareMap.get(DcMotor.class, "Rightlift");
            LeftHang = hardwareMap.get(DcMotorEx.class, "Lefthang");
            RightHang = hardwareMap.get(DcMotorEx.class, "Righthang");
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

                    lastliftpos = 3000;

                    Leftlift.setTargetPosition(lastliftpos);
                    Rightlift.setTargetPosition(lastliftpos);
                    Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Leftlift.setPower(1);
                    Rightlift.setPower(1);




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
                Leftlift.setPower(0.7);
                Rightlift.setPower(0.7);




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
                LeftHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        Pose2d initialPose = new Pose2d(35, 61.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        Action basket1;
        Action basket1depo;
        Action getblock2;
        Action basket2;
        Action getblock3;
        Action basket3;
        Action getblock4;
        Action block4turn;
        Action basket4;


         basket1 = drive.actionBuilder(initialPose)
                        .strafeToLinearHeading(new Vector2d(53,51), Math.toRadians(225))
                                .build();
         basket1depo = drive.actionBuilder(new Pose2d(53,51,Math.toRadians(225)))
                 .strafeTo(new Vector2d(56,53.5))
                         .build();
         getblock2 = drive.actionBuilder(new Pose2d(56,53.5,Math.toRadians(225)))
                 .strafeToLinearHeading(new Vector2d(42,52.25),Math.toRadians(285))
                         .build();
        basket2 = drive.actionBuilder(new Pose2d(42, 52.25, Math.toRadians(285)))
                .strafeToLinearHeading(new Vector2d(55.25,54.75), Math.toRadians(225))
                .build();

        getblock3 = drive.actionBuilder(new Pose2d(55.25,54.75,Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(55.25, 51), Math.toRadians(271))
                        .build();

        basket3 = drive.actionBuilder(new Pose2d(55.25, 51, Math.toRadians(271)))
                .strafeToLinearHeading(new Vector2d(54, 54.5), Math.toRadians(225))
                        .build();
        getblock4 = drive.actionBuilder(new Pose2d(54,54.5, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(47, 46), Math.toRadians(315))
                        .build();
        block4turn = drive.actionBuilder(new Pose2d(47,46, Math.toRadians(315)))
                .strafeToLinearHeading(new Vector2d(45, 46), Math.toRadians(270))
                .build();


        //init actions
        Actions.runBlocking(
                new SequentialAction(
                        intake.armstart(),
                        intake.inclawretract(),
                        outtake.outarmdown(),
                        outtake.outclawextend(),
                        intake.horizontalretraction(),
                        intake.inwristzero()
                )
        );
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
        waitForStart();
        if (opModeIsActive()) {

            Actions.runBlocking(
                    new SequentialAction(
                            intake.horizontalhalfextension(),

                            basket1,
                            lift.liftup(),
                            outtake.outarmup()

                    )
            );
            Actions.runBlocking(new SleepAction(1.5));
            Actions.runBlocking(new SequentialAction(
                    basket1depo,
                    outtake.outclawretract(),
                    outtake.outarmdown(),
                    lift.liftdown(),
                    getblock2,
                    intake.horizontalfullextension(),
                    intake.inarmup()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inarmdown()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inclawextend()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inarmback()
            ));
            Actions.runBlocking(new SleepAction(0.75));
            Actions.runBlocking(new SequentialAction(
                    intake.horizontalretraction()
            ));
            Actions.runBlocking(new SleepAction(0.75));
            Actions.runBlocking(new SequentialAction(
                    outtake.outclawextend()
            ));
            Actions.runBlocking(new SleepAction(0.25));
            Actions.runBlocking(new SequentialAction(
                    intake.inclawretract(),
                    intake.horizontalfullextension(),
                    intake.inarmup()
            ));
            Actions.runBlocking(new SleepAction(0.25));
            Actions.runBlocking(new SequentialAction(
                    lift.liftup(),
                    outtake.outarmup()
            ));
            Actions.runBlocking(new SleepAction(1));
            Actions.runBlocking(new SequentialAction(
                    basket2,
                    outtake.outclawretract()));
                    Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    getblock3,
                    lift.liftdown(),
                    outtake.outarmdown(),
                    intake.inarmup()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inarmdown()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inclawextend()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inarmback()
            ));
            Actions.runBlocking(new SleepAction(0.75));
            Actions.runBlocking(new SequentialAction(
                    intake.horizontalretraction()
            ));
            Actions.runBlocking(new SleepAction(0.75));
            Actions.runBlocking(new SequentialAction(
                    outtake.outclawextend()
            ));
            Actions.runBlocking(new SleepAction(0.25));
            Actions.runBlocking(new SequentialAction(
                    intake.inclawretract(),
                    intake.horizontalhalfextension()
            ));
            Actions.runBlocking(new SleepAction(1));
            Actions.runBlocking(new SequentialAction(
                    lift.liftup(),
                    outtake.outarmup(),
                    intake.horizontalfullextension(),
                    intake.inarmup(),
                    intake.wristmove()
            ));
            Actions.runBlocking(new SleepAction(2));
            Actions.runBlocking(new SequentialAction(
                    basket3,
                    outtake.outclawretract()
                            ));
                    Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    getblock4,
                    lift.liftdown(),
                    outtake.outarmdown()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inarmdown()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inclawextend()
            ));
            Actions.runBlocking(new SleepAction(0.5));
            Actions.runBlocking(new SequentialAction(
                    intake.inarmup(),
                    block4turn,
                    intake.inarmback(),
                    intake.inwristzero()
            ));
            Actions.runBlocking(new SleepAction(0.75));
            Actions.runBlocking(new SequentialAction(
                    intake.horizontalretraction()
            ));
            Actions.runBlocking(new SleepAction(0.75));
            Actions.runBlocking(new SequentialAction(
                    outtake.outclawextend()
            ));
            Actions.runBlocking(new SleepAction(0.25));
            Actions.runBlocking(new SequentialAction(
                    intake.inclawretract(),
                    intake.horizontalhalfextension()
            ));
            Actions.runBlocking(new SleepAction(1));



            sleep(30000);



        }
    }
}

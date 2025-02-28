package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
    public class InWristRight implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (inwristpos < 0.71 ){
                inwristpos = inwristpos + 0.05;
                inwrist.setPosition(inwristpos);
            }


            return false;
        }

    }
    public Action inwristright(){
        return new InWristRight();
    }

    public class InWristLeft implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (inwristpos > 0.03 ){
                inwristpos = inwristpos - 0.05;
                inwrist.setPosition(inwristpos);
            }


            return false;
        }

    }
    public Action inwristleft(){
        return new InWristLeft();
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
        public DcMotorEx Leftlift;
        public DcMotorEx Rightlift;
        int lastliftpos;
        double leftliftcurrent;
        double rightliftcurrent;



        public Lift(HardwareMap hardwareMap){
            Leftlift = hardwareMap.get(DcMotorEx.class, "Leftlift");
            Rightlift = hardwareMap.get(DcMotorEx.class, "Rightlift");
            Leftlift.setDirection(DcMotorEx.Direction.REVERSE);
            Rightlift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Leftlift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        }

        public class LiftUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(gamepad2.dpad_up){
                    lastliftpos = 3000;

                    Leftlift.setTargetPosition(lastliftpos);
                    Rightlift.setTargetPosition(lastliftpos);
                    Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Leftlift.setPower(1);
                    Rightlift.setPower(1);


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
                return false;
            }

        }
        public Action liftreset(){
            return new LiftReset();
        }
        public class LiftCurrent implements Action{


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                leftliftcurrent = Leftlift.getCurrent(CurrentUnit.AMPS);
                rightliftcurrent = Rightlift.getCurrent(CurrentUnit.AMPS);
                telemetry.addData("Leftliftcurrent", leftliftcurrent);
                telemetry.addData("Rightliftcurrent", rightliftcurrent);
                telemetry.addData("TargetPos", lastliftpos);
                telemetry.addData("LeftCurrPos", Leftlift.getCurrentPosition());
                telemetry.addData("RightCurrPos", Rightlift.getCurrentPosition());
                telemetry.addData("LeftMotorpower", Leftlift.getController().getMotorPower(0));
                telemetry.addData("RightMotorpower", Rightlift.getController().getMotorPower(1));
                telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                return false;
            }

        }
        public Action liftcurrent(){
            return new LiftCurrent();
        }



    }








    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        float forwardpos;
        float horizontalpos;
        float headingpos;
        boolean liftdown;
        boolean moveoutarmup;
        boolean intakein;
        double drivespeed;
        double liftpower;
        boolean moveinarm;
        boolean moveintake;
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

        moveinarm = false;
        moveoutarmup = false;
        liftdown = true;
        intakein = true;
        moveintake = false;
        ElapsedTime timer1 = null;
        ElapsedTime timer2 = null;
        ElapsedTime timer3 = null;
        ElapsedTime timecheck = null;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            // set outtake arm on tele start
            Actions.runBlocking(
                    new SequentialAction(
                        outtake.outarmdown(),

                            outtake.outclawretract(),
                            lift.liftreset()
                    )
            );
            timer2 = new ElapsedTime();
            timer1 = new ElapsedTime();
            timer3 = new ElapsedTime();
            while (opModeIsActive()) {
                // Put loop blocks here.
                // driving blocks
                timecheck = new ElapsedTime();
                forwardpos = gamepad1.left_stick_y;
                horizontalpos = -gamepad1.left_stick_x;
                headingpos = -gamepad1.right_stick_x;
                Actions.runBlocking(new SequentialAction(lift.liftcurrent()));

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
                if (gamepad1.right_bumper){
                    Actions.runBlocking(intake.inwristright());
                }
                if (gamepad1.left_bumper){
                    Actions.runBlocking(intake.inwristleft());
                }
                if (gamepad1.start){
                    Actions.runBlocking(intake.inwristreset());
                }



                // return intake for transfer
                if (gamepad1.back) {
                    intakein = true;
                    Actions.runBlocking(
                            new SequentialAction(
                                    intake.inarmback(),
                                    intake.inwristzero()));
                    timer1 = new ElapsedTime();
                    moveinarm = true;

                }
                if (timer1.time() > 0.85 && moveinarm) {
                    Actions.runBlocking(new SequentialAction(intake.horizontalretraction()));
                    moveinarm = false;
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
                    Actions.runBlocking(new SequentialAction(lift.liftup()));
                    timer2 = new ElapsedTime();
                    moveoutarmup = true;
                }
                if (timer2.time() > 0.2 && moveoutarmup) {
                    Actions.runBlocking(new SequentialAction(outtake.outarmup()));
                    moveoutarmup = false;
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
                                    outtake.outclawretract()

                            )
                    );

                }
                // open/ close outtake claw
                if (gamepad2.x && liftdown) {


                    Actions.runBlocking(new SequentialAction(outtake.outclawextend(),intake.inclawretract()));

                    moveintake = true;

                }
                if (timer3.time() >= 0.05 && moveintake){
                    Actions.runBlocking(intake.horizontalhalfextension());
                    moveintake = false;
                }

                if (gamepad2.y) {
                    Actions.runBlocking(outtake.outclawretract());
                }

                telemetry.addData("looptime", timecheck.time());
                timecheck.reset();


                telemetry.update();
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp(name = "VWFCBasketSide")
public class VWFCBasketSide extends LinearOpMode {
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
            rightext.setPosition(0.292);
            leftext.setPosition(0.292);
            return false;
        }
    }
    public Action horizontalretraction(){
        return new HorizontalRetraction();
    }

    public class HorizontalHalfExtension implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rightext.setPosition(0.31);
            leftext.setPosition(0.31);
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
            outarmtransferpos = 0.762;
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
                outbelt.setPosition(0.555);
                return false;
            }

        }
        public Action outarmdown(){
            return new OutArmDown();
        }

        public class OutArmUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outarm.setPosition(outarmtransferpos - 0.165);
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

        public class LiftUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(gamepad2.dpad_up){ //&& liftlistnum > 0 && liftlistnum < 3) {
                    lastliftpos = 3000; //liftposes.get(liftlistnum + 1);

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

        public class LiftUpHang implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                lastliftpos = 1000;

                Leftlift.setTargetPosition(lastliftpos);
                Rightlift.setTargetPosition(lastliftpos);
                Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Leftlift.setPower(0.5);
                Rightlift.setPower(0.5);



                return false;
            }

        }
        public Action liftuphang(){
            return new LiftUpHang();
        }
        public class LiftDownHang implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                lastliftpos = 375;

                Leftlift.setTargetPosition(lastliftpos);
                Rightlift.setTargetPosition(lastliftpos);
                Leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Leftlift.setPower(.5);
                Rightlift.setPower(.5);



                return false;
            }

        }
        public Action liftdownhang(){
            return new LiftDownHang();
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
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");


        // Put initialization blocks here.
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        inwristpos = 0.5;
        lastliftpos = 2000;
        liftamount = 250;
        outclawext = false;
        liftdown = true;
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

                double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double adjustedHorizontalpos = -forwardpos * Math.sin(heading) + horizontalpos * Math.cos(heading);
                double adjustedForwardpos = forwardpos * Math.cos(heading) + horizontalpos * Math.sin(heading);

                RightFront.setPower((-headingpos + (adjustedForwardpos - adjustedHorizontalpos)) * 0.5);
                RightBack.setPower((-headingpos + adjustedForwardpos + adjustedHorizontalpos) * 0.5);
                LeftFront.setPower((headingpos + adjustedForwardpos + adjustedHorizontalpos) * 0.5);
                LeftBack.setPower((headingpos + (adjustedForwardpos - adjustedHorizontalpos)) * 0.5);
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
                    Actions.runBlocking(intake.horizontalfullextension());
                }
                if (gamepad1.left_trigger >= 0.5) {
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
                    sleep(50);
                    Actions.runBlocking(new SequentialAction(
                            intake.inclawretract()));
                            sleep(50);
                    Actions.runBlocking(new SequentialAction(
                            intake.horizontalhalfextension()));
                    sleep(200);
                    Actions.runBlocking(new SequentialAction(
                            intake.transfer()));
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
                if (gamepad2.a) {
                    Actions.runBlocking(lift.liftuphang());
                }
                if (gamepad2.b) {
                    Actions.runBlocking(lift.liftdownhang());
                }

                telemetry.update();
            }
        }
    }
}

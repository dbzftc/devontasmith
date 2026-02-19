package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.opmodes._redside.servooffset;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;
import org.firstinspires.ftc.teamcode.opmodes._blueside;

@Config
@Autonomous(name = "_redautonew", group = "Autonomous")
public class _redautoootbrizz extends DbzOpMode {

    private final ElapsedTime detectionTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime jawntimer = new ElapsedTime();

    public static double startX = 128.200;
    public static double startY = 111.700;
    public static double startHeadingDeg = 90.0;

    public static double xend = 130;

    public static double gate1y = 59.21;
    public static double gate1x = 132;
    public static double gate1h = 21;



    public static double gate2x = 132;
    public static double gate2y = 58.21;

    public static double gate2h = 21;

    public static double leftUpPush = 0.023;
    public static double rightUpPush = 0.00;
    public static boolean intakeatwall = false;

    public static double waitshoot1 = 1000;
    public static double waitshoot2 = 500;
    public static double waitgate1 = 300;

    public static double waitgate1andahalf = 1000;
    public static double waitgate2 = 2000;

    public static double holdClosePos = 0.12;
    public static double lock = 0.15;
    public static double targetX = 144.0;
    public static double targetY = 144.0;
    public static double holdOpenPos = 0.2;
    public static double leftPushShoot = 0.66;
    public static double rightPushShoot = 0.637;
    public static double leftPushIdle = 0.06;
    public static double rightPushIdle = 0.0577;
    public static double targetVelocity = -1350;
    public static double vkP = 4.8;
    public static double vkF = 1.32;
    public static double turretZeroDeg = 329;
    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.002;
    public static double turretMaxPower = 1;
    public static double threshold = 180;
    public static double threshold2 = 180;
    public static double hoodServoPos = 0.33;
    public static double dthresh = 4.4;

    private Servo rightpushServo, leftpushServo, holdServo, hoodServo;
    private DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;
    private AnalogInput turretEncoder;
    private VoltageSensor batteryVoltageSensor;
    private PIDController turretPID;
    private Follower follower;
    private Paths paths;
    private final ElapsedTime waitTimer = new ElapsedTime();
    private boolean shooting = false;
    private double waitms = 1000;
    private boolean detected = false;
    protected DistanceSensor sensor1, sensor2;
    private int state = 0;

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16;

        public Paths(Follower follower, double xend, double gate1x, double gate1y, double gate1h, double gate2x, double gate2y, double gate2h) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(128.2, 112.7), new Pose(111.44, 108.17), new Pose(99.18, 91.65))
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(99.18, 91.65), new Pose(89.79, 62.03), new Pose(103.95, 58.87))
            ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0)).build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(103.95, 58.87), new Pose(xend, 58.37))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(xend, 58.37), new Pose(93.06, 56.91), new Pose(96.27, 82.89))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(96.27, 82.89), new Pose(95.34, 52.71), new Pose(gate1x, gate1y))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gate1h)).build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(gate1x, gate1y), new Pose(gate2x, gate2y))
            ).setLinearHeadingInterpolation(Math.toRadians(gate1h), Math.toRadians(gate2h)).build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(gate2x, gate2y), new Pose(97.18, 61.36), new Pose(96.27, 82.89))
            ).setLinearHeadingInterpolation(Math.toRadians(gate2h), Math.toRadians(0)).build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(96.27, 82.89), new Pose(97.26, 61.22), new Pose(gate1x, gate1y))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gate1h)).build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(gate1x, gate1y), new Pose(122, 59.21))
            ).setLinearHeadingInterpolation(Math.toRadians(gate1h), Math.toRadians(gate2h)).build();

            Path10 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(122, 59.21), new Pose(97.32, 61.23), new Pose(96.27, 82.89))
            ).setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(0)).build();

            Path11 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(96.27, 82.89), new Pose(98.60, 84.29), new Pose(xend - 4.5, 83.65))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path12 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(xend, 83.65), new Pose(96.42, 82.90))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path13 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(96.42, 82.90), new Pose(89.41, 33.21), new Pose(108.47, 35.28))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path14 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(108.47, 35.28), new Pose(xend, 35.12))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path15 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(xend, 35.12), new Pose(110.67, 63.41), new Pose(96.42, 82.90))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path16 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(96.42, 82.90), new Pose(117.52, 82.68))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
        }
    }

    @Override
    public void opInit() {
        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        holdServo = hardwareMap.get(Servo.class, "holdServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        sensor1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        sensor2 = hardwareMap.get(DistanceSensor.class, "sensor2");
        intakeMotor = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;
        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        turretPID = new PIDController(turretKp, turretKi, turretKd);
        outtake1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2Motor.setDirection(DcMotorEx.Direction.FORWARD);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        paths = new Paths(follower, xend, gate1x, gate1y, gate1h, gate2x, gate2y, gate2h);
        holdServo.setPosition(holdOpenPos);
        leftpushServo.setPosition(lock);
        rightpushServo.setPosition(lock - servooffset);
        hoodServo.setPosition(hoodServoPos);
    }

    @Override
    public void opLoop() {
        follower.update();
        updateHoodAndVelocity();
        runFlywheelVelocityControl();
        runTurretAlwaysOn();

        double d1 = sensor1.getDistance(DistanceUnit.CM);
        double d2 = sensor2.getDistance(DistanceUnit.CM);
        if (!(d1 < dthresh || d2 < dthresh)) {
            detectionTimer.reset();
            detected = false;
        }
        detected = detectionTimer.milliseconds() > 500;


        if(intakeatwall){}
        else if (shooting) {
            leftpushServo.setPosition(leftPushShoot);
            rightpushServo.setPosition(rightPushShoot);
            holdServo.setPosition(holdOpenPos);
            intakeMotor.setPower(-1.0);
        } else if (follower.isBusy()) {
            leftpushServo.setPosition(leftPushIdle);
            rightpushServo.setPosition(rightPushIdle);
            intakeMotor.setPower(-1.0);
            holdServo.setPosition(holdClosePos);
        } else if (detected &&!shooting) {
            jawntimer.reset();
            leftpushServo.setPosition(lock);
            rightpushServo.setPosition(lock - _blueside.servooffset);
            if(jawntimer.milliseconds()<200) {
                intakeMotor.setPower(1);
            }
            else if (jawntimer.milliseconds()>200){
                intakeMotor.setPower(0);
            }
            holdServo.setPosition(holdClosePos);
        } else {
            intakeMotor.setPower(0);

        }


        switch (state) {
            case 0:
                follower.followPath(paths.Path1);
                state = 1;
                break;
            case 1:
                if (!follower.isBusy()) beginWait(waitshoot1, true, 2);
                break;
            case 2:
                if (waitDone()) {
                    follower.followPath(paths.Path2);
                    state = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    state = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    state = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) beginWait(waitshoot2, true, 6);
                break;
            case 6:
                if (waitDone()) {
                    follower.followPath(paths.Path5);
                    state = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) beginWait(waitgate1, false, 8);
                intakeatwall=true;
                break;
            case 8:
                if (waitDone()) {
                    follower.followPath(paths.Path6);
                    intakeatwall=true;
                    state = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) beginWait(waitgate2, false, 10);
                intakeatwall= true;
                break;
            case 10:
                if (waitTimer.milliseconds() >= waitgate2 || detected) {
                    shooting = false;
                    intakeatwall=true;
                    follower.followPath(paths.Path7);
                    state = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) beginWait(waitshoot2, true, 12);
                intakeatwall=false;
                break;
            case 12:
                if (waitDone()) {
                    follower.followPath(paths.Path8);
                    state = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) beginWait(waitgate1andahalf, false, 14);
                intakeatwall=true;
                break;
            case 14:
                if (waitDone()) {
                    follower.followPath(paths.Path9);
                    intakeatwall=true;
                    state = 15;
                }
                break;
            case 15:
                if (!follower.isBusy()) beginWait(waitgate2, false, 16);
                intakeatwall=true;

                break;
            case 16:
                if (waitTimer.milliseconds() >= waitgate2 || detected) {
                    shooting = false;
                    intakeatwall=true;
                    follower.followPath(paths.Path10);
                    state = 17;
                }
                break;
            case 17:
                if (!follower.isBusy()) beginWait(500, true, 18);
                intakeatwall=false;
                break;
            case 18:
                if (waitDone()) {
                    follower.followPath(paths.Path11);
                    state = 19;
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path12);
                    state = 20;
                }
                break;
            case 20:
                if (!follower.isBusy()) beginWait(500, true, 21);
                break;
            case 21:
                if (waitDone()) {
                    follower.followPath(paths.Path13);
                    state = 22;
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path14);
                    state = 23;
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path15);
                    state = 24;
                }
                break;
            case 24:
                if (!follower.isBusy()) beginWait(500, true, 25);
                break;
            case 25:
                if (waitDone()) {
                    follower.followPath(paths.Path16);
                    state = 26;
                }
                break;
        }
    }

    private void updateHoodAndVelocity() {
        if (state >= 30) targetVelocity = -500;
    }

    private void beginWait(double ms, boolean shoot, int nxt) {
        waitms = ms;
        shooting = shoot;
        if (shoot) shootTimer.reset();
        waitTimer.reset();
        state = nxt;
    }

    private boolean waitDone() {
        if (waitTimer.milliseconds() >= waitms) {
            shooting = false;
            return true;
        }
        return false;
    }

    private void runFlywheelVelocityControl() {
        double cur = outtake2Motor.getVelocity();
        double max = outtake2Motor.getMotorType().getMaxRPM() * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;
        double power = vkP * ((targetVelocity - cur) / max) + vkF * (targetVelocity / max) * (12.0 / Math.max(10.5, batteryVoltageSensor.getVoltage()));
        outtake1Motor.setPower(Math.max(-1.0, Math.min(1.0, power)));
        outtake2Motor.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    private void runTurretAlwaysOn() {
        Pose p = follower.getPose();
        if (p == null) return;
        double fieldAngle = Math.atan2(targetY - p.getY(), targetX - p.getX());
        double des = Math.toDegrees(fieldAngle - p.getHeading());
        des = angleWrap(des);
        if (des > threshold2) des = threshold2;
        if (des < -threshold) des = -threshold;
        double out = turretPID.calculate(getTurretAngleDeg(), des);
        turret.setPower(Math.max(-turretMaxPower, Math.min(turretMaxPower, out)));
    }

    private double getTurretAngleDeg() {
        return angleWrap((turretEncoder.getVoltage() / turretEncoder.getMaxVoltage()) * 360.0 - turretZeroDeg);
    }

    private double angleWrap(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
    }

    @Override
    public void opLoopHook() {}

    @Override
    public void opTeardown() {
        org.firstinspires.ftc.teamcode.opmodes.PoseCache.lastPose = follower.getPose();
    }
}
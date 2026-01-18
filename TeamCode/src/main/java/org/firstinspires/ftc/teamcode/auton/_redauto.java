package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.auton._blueauto.holdClosePos;
import static org.firstinspires.ftc.teamcode.opmodes._blueside.servooffset;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.TelemetryManager;
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

@Config
@Autonomous(name = "_redauto", group = "Autonomous")
public class _redauto extends DbzOpMode {

    private final ElapsedTime detectionTimer = new ElapsedTime();

    public static double startX = 128.200;
    public static double startY = 111.700;
    public static double startHeadingDeg = 90.0;



    public static double xend = 130;

    public static double gate1y = 58.5;
    public static double gate1x = 126.5;
    public static double gate1h = 30;
    public static double gate2x = 133;
    public static double gate2y = 56.5;
    public static double gate2h = 28;
    public static double waitshoot1 = 1000;

    public static double waitshoot2 = 500;

    public static double waitgate1 = 150;
    public static double waitgate2 = 1000;

    //132, 58.5, 25
    //133, 50, 28















    public static double holdClosePos = 0.05;
    public static double lock = 0.15;

    public static double targetX = 140.0;
    public static double targetY = 144.0;

    public static double holdOpenPos = 0.2;

    public static double leftPushShoot = 0.66;
    public static double rightPushShoot = 0.683;

    public static double leftPushIdle = 0.06;
    public static double rightPushIdle = 0.083;

    public static double targetVelocity = -1340;
    public static double vkP = 4.8;
    public static double vkF = 1.26;
//132, 58.5, 25
    //133, 50, 28
    public static double turretZeroDeg = 295;
    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.002;
    public static double turretMaxPower = 1;
    public static double threshold = 90;
    public static double threshold2 = 160;

    public static double turretPivotForwardIn = 0.0;
    public static double turretPivotLeftIn = 0.0;

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
    public static TelemetryManager telemetryM;
    protected DistanceSensor sensor1, sensor2;
    protected Servo light;

    private int state = 0;

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;
        public PathChain Path16;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(128.200, 112.700),
                            new Pose(111.438, 108.171),
                            new Pose(99.175, 91.653)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(99.175, 91.653),
                            new Pose(89.789, 62.031),
                            new Pose(103.946, 58.870)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0)).build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(103.946, 58.870),
                            new Pose(xend, 58.369)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(xend, 58.369),
                            new Pose(93.064, 56.912),
                            new Pose(96.274, 82.893)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(96.274, 82.893),
                            new Pose(95.341, 52.714),
                            new Pose(gate1x, gate1y)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gate1h)).build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(gate1x, gate1y),
                            new Pose(gate2x, gate2y)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(gate1h), Math.toRadians(gate2h)).build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(gate2x, gate2y),
                            new Pose(97.177, 61.362),
                            new Pose(96.274, 82.893)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(gate2h), Math.toRadians(0)).build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(96.274, 82.893),
                            new Pose(97.256, 61.222),
                            new Pose(gate1x, gate1y)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gate1h)).build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(gate1x, gate1y),
                            new Pose(gate2x, gate2y)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(gate1h), Math.toRadians(gate2h)).build();

            Path10 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(gate2x, gate2y),
                            new Pose(97.324, 61.233),
                            new Pose(96.274, 82.893)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(0)).build();

            Path11 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(96.274, 82.893),
                            new Pose(98.598, 84.288),
                            new Pose(xend-6.5, 83.648)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path12 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(xend, 83.648),
                            new Pose(96.421, 82.904)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path13 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(96.421, 82.904),
                            new Pose(89.413, 33.210),
                            new Pose(108.468, 35.275)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path14 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(108.468, 35.275),
                            new Pose(xend, 35.119)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path15 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(xend, 35.119),
                            new Pose(110.670, 63.406),
                            new Pose(96.421, 82.904)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Path16 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(96.421, 82.904),
                            new Pose(117.522, 82.678)
                    )
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
        light = hardwareMap.get(Servo.class, "light");

        intakeMotor = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);

        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        turretPID = new PIDController(turretKp, turretKi, turretKd);

        outtake1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2Motor.setDirection(DcMotorEx.Direction.FORWARD);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        paths = new Paths(follower);

        holdServo.setPosition(holdOpenPos);
        leftpushServo.setPosition(leftPushIdle);
        rightpushServo.setPosition(rightPushIdle);
        hoodServo.setPosition(hoodServoPos);
    }

    @Override
    public void opLoop() {

        follower.update();
        updateHoodAndVelocity();
        runFlywheelVelocityControl();
        runTurretAlwaysOn();

        double dist1 = sensor1.getDistance(DistanceUnit.CM);
        double dist2 = sensor2.getDistance(DistanceUnit.CM);

        if (!(dist1 < dthresh || dist2 < dthresh)) {
            detectionTimer.reset();
            detected = false;

        }
        detected = detectionTimer.milliseconds() > 500;






        holdServo.setPosition(holdOpenPos);

        if (shooting) {
            leftpushServo.setPosition(leftPushShoot);
            rightpushServo.setPosition(rightPushShoot);
            holdServo.setPosition(holdOpenPos);
            intakeMotor.setPower(-1);
        } else if (detected) {
            leftpushServo.setPosition(lock);
            rightpushServo.setPosition(lock - servooffset);
            intakeMotor.setPower(1);



        } else {
            leftpushServo.setPosition(leftPushIdle);
            rightpushServo.setPosition(rightPushIdle);
            intakeMotor.setPower(-1);
        }
        switch (state) {
            case 0:
                follower.followPath(paths.Path1);
                state = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    beginWait(waitshoot1, true, 2);
                }
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
                if (!follower.isBusy()) {
                    beginWait(waitshoot2, true, 6);
                }
                break;

            case 6:
                if (waitDone()) {
                    follower.followPath(paths.Path5);
                    state = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    beginWait(waitgate1, false, 8);
                }
                break;

            case 8:
                if (waitDone()) {
                    follower.followPath(paths.Path6);
                    state = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    beginWait(waitgate2, false, 10);
                }
                break;

            case 10:
                if (waitTimer.milliseconds() >= waitgate2 || detected) {
                    shooting = false;
                    follower.followPath(paths.Path7);
                    state = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    beginWait(waitshoot2, true, 12);
                }
                break;

            case 12:
                if (waitDone()) {
                    follower.followPath(paths.Path8);
                    state = 13;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    beginWait(waitgate1, false, 14);
                }
                break;

            case 14:
                if (waitDone()) {
                    follower.followPath(paths.Path9);
                    state = 15;
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    beginWait(waitgate2, false, 16);
                }
                break;



            case 16:
                if (waitTimer.milliseconds() >= waitgate2 || detected) {
                    shooting = false;
                    follower.followPath(paths.Path10);
                    state = 17;
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    beginWait(500, true, 18);
                }
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
                if (!follower.isBusy()) {
                    beginWait(500, true, 21);
                }
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
                if (!follower.isBusy()) {
                    beginWait(500, true, 25);
                }
                break;

            case 25:
                if (waitDone()) {
                    follower.followPath(paths.Path16);
                    state = 26;
                }
                break;

            case 26:
                if (!follower.isBusy()) {
                    state = 27;
                }
                break;

            case 27:
                break;
        }
    }

    private void updateHoodAndVelocity() {
        boolean active = state < 30;
        if (!active) {
            targetVelocity = -500;
            hoodServo.setPosition(hoodServoPos);
            return;
        }
        Pose pose = follower.getPose();
        if (pose == null) {
            targetVelocity = -500;
            hoodServo.setPosition(hoodServoPos);
            return;
        }
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
    }

    private void beginWait(double ms, boolean doshoot, int nextState) {
        waitms = ms;
        shooting = doshoot;
        waitTimer.reset();
        state = nextState;
    }

    private boolean waitDone() {
        if (waitTimer.milliseconds() >= waitms) {
            shooting = false;
            return true;
        }
        return false;
    }

    private void runFlywheelVelocityControl() {
        double currentVelocity = outtake2Motor.getVelocity();
        double maxVelocity = outtake2Motor.getMotorType().getMaxRPM() * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;
        double normalizedError = (targetVelocity - currentVelocity) / maxVelocity;
        double pTerm = vkP * normalizedError;
        double feedforward = vkF * (targetVelocity / maxVelocity);
        double batteryVoltage = Math.max(10.5, batteryVoltageSensor.getVoltage());
        feedforward *= 12.0 / batteryVoltage;
        double power = Math.max(-1.0, Math.min(1.0, pTerm + feedforward));
        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);
    }

    private void runTurretAlwaysOn() {
        Pose pose = follower.getPose();
        if (pose == null) return;
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double fieldAngle = Math.atan2(dy, dx);
        double desiredDeg = Math.toDegrees(fieldAngle - pose.getHeading());
        desiredDeg = angleWrap(desiredDeg);
        if (desiredDeg > threshold2) desiredDeg = threshold2;
        if (desiredDeg < -threshold) desiredDeg = -threshold;
        turretPID.setPID(turretKp, turretKi, turretKd);
        double currentDeg = getTurretAngleDeg();
        double out = turretPID.calculate(currentDeg, desiredDeg);
        if (out > turretMaxPower) out = turretMaxPower;
        if (out < -turretMaxPower) out = -turretMaxPower;
        turret.setPower(out);


    }


    private double getTurretAngleDeg() {
        double voltage = turretEncoder.getVoltage();
        double angle = (voltage / turretEncoder.getMaxVoltage()) * 360.0;
        angle -= turretZeroDeg;
        return angleWrap(angle);
    }

    private double angleWrap(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    @Override
    public void opLoopHook() {
    }

    @Override
    public void opTeardown() {
        org.firstinspires.ftc.teamcode.opmodes.PoseCache.lastPose = follower.getPose();
    }
}

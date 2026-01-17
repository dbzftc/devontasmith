package org.firstinspires.ftc.teamcode.auton;

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

import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@Autonomous(name = "autored", group = "Autonomous")
public class autored extends DbzOpMode {

    // Start pose updated to match new Path1 start
    public static double startX = 128.200;
    public static double startY = 112.700;
    public static double startHeadingDeg = 90;

    /// IMPORTANT VALUES PAY ATTENTION BUBUTANG
    public static double gatx= 130.5;
    public static double gaty= 61.5;
    public static double gath= 37.0;
    public static double xend = 124;
    public static double v = 2;

    public static double shootx = 85;
    public static double shooty = 85;
    /// IMPORTANT

    public static double targetX = 144.0;
    public static double targetY = 144.0;

    public static double holdOpenPos = 0.2;
    public static double holdClosePos = 0.1;

    public static double leftPushShoot = 0.66;
    public static double rightPushShoot = 0.69;

    public static double leftPushIdle = 0.06;
    public static double rightPushIdle = 0.09;

    public static double targetVelocity = -1500;
    public static double vkP = 4.8;
    public static double vkF = 1.08;

    public static double turretZeroDeg = 295;
    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.002;
    public static double turretMaxPower = 1;
    public static double threshold = 150;

    public static double turretPivotForwardIn = 0.0;
    public static double turretPivotLeftIn = 0.0;

    public static double hoodServoPos = 0.55;

    private Servo rightpushServo, leftpushServo, holdServo, hoodServo;
    private DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;
    private AnalogInput turretEncoder;
    private VoltageSensor batteryVoltageSensor;

    private PIDController turretPID;

    private Follower follower;
    private Paths paths;

    private final ElapsedTime waitTimer = new ElapsedTime();
    private boolean shooting = false;
    private double waitms = 0;

    private static final double waitShoot = 500;
    private static final double waitShoot1 = 500;

    public static TelemetryManager telemetryM;
    protected DistanceSensor sensor1, sensor2;
    protected Servo light;

    public static double dthresh = 2.0;

    private static final double waitGate  = 1500;
    private static final double waitTiny  = 100;

    private int state = 0;

    // NEW PATHS CLASS (15 paths) — from your latest pathing
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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.200, 112.700),
                                    new Pose(111.438, 108.171),
                                    new Pose(99.175, 91.653)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(99.175, 91.653),
                                    new Pose(89.789, 62.031),
                                    new Pose(103.946, 58.870)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(103.946, 58.870),
                                    new Pose(129.542, 58.369)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.542, 58.369),
                                    new Pose(93.064, 56.912),
                                    new Pose(96.274, 82.893)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(96.274, 82.893),
                                    new Pose(95.341, 52.714),
                                    new Pose(129.860, 60.979)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(30))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.860, 60.979),
                                    new Pose(125.687, 55.390),
                                    new Pose(131.012, 57.794)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(33))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131.012, 57.794),
                                    new Pose(97.177, 61.362),
                                    new Pose(96.274, 82.893)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(33), Math.toRadians(40))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(96.274, 82.893),
                                    new Pose(97.256, 61.222),
                                    new Pose(129.860, 60.979)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(30))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.860, 60.979),
                                    new Pose(125.339, 55.290),
                                    new Pose(131.553, 57.974)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(33))
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131.553, 57.974),
                                    new Pose(97.324, 61.233),
                                    new Pose(96.274, 82.893)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(33), Math.toRadians(0))
                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(96.274, 82.893),
                                    new Pose(98.598, 84.288),
                                    new Pose(126.910, 83.648)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.910, 83.648),
                                    new Pose(96.421, 82.904)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(33))
                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(96.421, 82.904),
                                    new Pose(89.413, 33.210),
                                    new Pose(108.468, 35.275)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(33), Math.toRadians(0))
                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(108.468, 35.275),
                                    new Pose(131.431, 35.119)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path15 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131.431, 35.119),
                                    new Pose(110.670, 63.406),
                                    new Pose(91.944, 110.907)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();
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

        // If 'light' is also throwing errors, initialize it here too:
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

        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(leftPushIdle);
        rightpushServo.setPosition(rightPushIdle);
        hoodServo.setPosition(hoodServoPos);
    }

    @Override
    public void opLoop() {
        intakeMotor.setPower(-1);
        follower.update();
        updateHoodAndVelocity();
        runFlywheelVelocityControl();
        runTurretAlwaysOn();

        double dist1 = sensor1.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
        double dist2 = sensor2.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);

        boolean detected1 = dist1 < dthresh;
        boolean detected2 = dist2 < dthresh;

        if (detected1 || detected2) {
            light.setPosition(0.5); // Example: Set to a specific color (e.g., Green or Yellow)
        } else {
            light.setPosition(0.0); // Off or Default color
        }

        holdServo.setPosition(holdOpenPos);
        leftpushServo.setPosition(shooting ? leftPushShoot : leftPushIdle);
        rightpushServo.setPosition(shooting ? rightPushShoot : rightPushIdle);

        switch (state) {
            case 0:
                follower.followPath(paths.Path1);
                state = 1;
                break;
            case 1:
                // When Path 1 finishes, wait 500ms WITHOUT shooting (servos stay idle)
                if (!follower.isBusy()) {
                    beginWait(500, false, 21); // Using 21 as a temporary state for 1.5
                }
                break;

            case 21: // Effectively State 1.5
                // After 500ms is up, start the actual shooting (servos push) for the remainder
                if (waitDone()) {
                    beginWait(waitShoot1, true, 2);
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
                if (!follower.isBusy()) beginWait(waitShoot, true, 5);
                break;
            case 5:
                if (waitDone()) {
                    follower.followPath(paths.Path4);
                    state = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) beginWait(waitGate, false, 7);
                holdServo.setPosition(holdClosePos);
                break;
            case 7:
                if (waitDone() || detected1 || detected2) {
                    follower.followPath(paths.Path5);
                    holdServo.setPosition(holdOpenPos);
                    state = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) beginWait(waitShoot, true, 9);
                break;
            case 9:
                if (waitDone()) {
                    follower.followPath(paths.Path6);
                    state = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) beginWait(waitGate, false, 11);
                holdServo.setPosition(holdClosePos);
                break;
            case 11:
                if (waitDone() || detected1 || detected2) {
                    follower.followPath(paths.Path7);
                    holdServo.setPosition(holdOpenPos);
                    state = 12;
                }
                break;
            case 12:
                // New Wait added after Path 7
                if (!follower.isBusy()) beginWait(waitShoot, true, 13);
                break;
            case 13:
                if (waitDone()) {
                    follower.followPath(paths.Path8);
                    state = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    state = 15;
                }
                break;
            case 15:
                if (!follower.isBusy()) beginWait(waitShoot, true, 16);
                break;
            case 16:
                if (waitDone()) {
                    follower.followPath(paths.Path10);
                    state = 17;
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11);
                    state = 18;
                }
                break;
            case 18:
                if (!follower.isBusy()) beginWait(waitShoot, true, 19);
                break;
            case 19:
                if (waitDone()) {
                    follower.followPath(paths.Path12);
                    state = 20;
                }
                break;
            case 20:
                break;
        }
    }

    private void updateHoodAndVelocity() {
        boolean active = state < 25;
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
        // you can use distance to dynamically tune hood/velocity if you want
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
        if (desiredDeg > threshold) desiredDeg = threshold;
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

    @Override public void opLoopHook() {}

    @Override
    public void opTeardown() {
        org.firstinspires.ftc.teamcode.opmodes.PoseCache.lastPose = follower.getPose();
    }
}

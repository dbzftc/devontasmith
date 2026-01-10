package org.firstinspires.ftc.teamcode.auton;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@Autonomous(name = "namitparikh_new", group = "Autonomous")
public class namitparikh_new extends DbzOpMode {

    public static double startX = 39.778;
    public static double startY = 135.529;
    public static double startHeadingDeg = 90;

    // Field target (for turret + hood regression)
    public static double targetX = 0.0;
    public static double targetY = 144.0;

    public static double holdOpenPos = 0.2;
    public static double holdClosePos = 0.06;

    public static double leftPushShoot = 0.66;
    public static double rightPushShoot = 0.69;

    public static double leftPushIdle = 0.06;
    public static double rightPushIdle = 0.09;

    public static double targetVelocity = -1400;
    public static double vkP = 4.8;
    public static double vkF = 1.07;

    public static double turretZeroDeg = 360;
    public static double turretKp = 0.014;
    public static double turretKi = 0.0;
    public static double turretKd = 0.001;
    public static double turretMaxPower = 0.30;
    public static double threshold = 175;

    public static double turretPivotForwardIn = 0.0;
    public static double turretPivotLeftIn = 0.0;

    public static double hoodServoPos = 0.33;

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

    // Timers (ms)
    private static final double waitShoot = 1000;   // places where you shoot
    private static final double waitGate  = 2000;   // gate waits (no shooting)
    private static final double waitTiny  = 100;    // tiny final wait

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

        public Paths(Follower f) {
            // Path1: start to first shooting point (59.244, 84.166)
            Path1 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(39.778, 135.529),
                                    new Pose(59.244, 84.166)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            // Path2: 59.244,84.166 -> 15.697,60.586
            Path2 = f.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.244, 84.166),
                                    new Pose(68.907, 57.651),
                                    new Pose(15.697, 60.586)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path3: reverse of Path2 (back to 59.244,84.166)
            Path3 = f.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.697, 60.586),
                                    new Pose(68.907, 57.651),
                                    new Pose(59.244, 84.166)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path4: 59.244,84.166 -> 11.979,59.646
            Path4 = f.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.244, 84.166),
                                    new Pose(34.717, 50.164),
                                    new Pose(11.979, 59.646)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path5: reverse of Path4 (back to 59.244,84.166)
            Path5 = f.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.979, 59.646),
                                    new Pose(34.717, 50.164),
                                    new Pose(59.244, 84.166)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path6: 59.244,84.166 -> 11.979,59.646 (second time, same as Path4)
            Path6 = f.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.244, 84.166),
                                    new Pose(34.717, 50.164),
                                    new Pose(11.979, 59.646)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path7: reverse of Path6 (back again to 59.244,84.166)
            Path7 = f.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.979, 59.646),
                                    new Pose(34.717, 50.164),
                                    new Pose(59.244, 84.166)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path8: 59.244,84.166 -> 13.394,84.033
            Path8 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.244, 84.166),
                                    new Pose(13.394, 84.033)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path9: reverse of Path8 (13.394,84.033 -> 59.244,84.166)
            Path9 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.394, 84.033),
                                    new Pose(59.244, 84.166)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path10: 59.244,84.166 -> 11.506,36.198
            Path10 = f.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.244, 84.166),
                                    new Pose(64.049, 31.024),
                                    new Pose(11.506, 36.198)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path11: reverse of Path10 (back to 59.244,84.166)
            Path11 = f.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.506, 36.198),
                                    new Pose(64.049, 31.024),
                                    new Pose(59.244, 84.166)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path12: final move 59.244,84.166 -> 58.985,104.514
            Path12 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.244, 84.166),
                                    new Pose(58.985, 104.514)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    @Override
    public void opInit() {
        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        holdServo = hardwareMap.get(Servo.class, "holdServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

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

        holdServo.setPosition(shooting ? holdOpenPos : holdClosePos);
        leftpushServo.setPosition(shooting ? leftPushShoot : leftPushIdle);
        rightpushServo.setPosition(shooting ? rightPushShoot : rightPushIdle);

        switch (state) {
            case 0:
                follower.followPath(paths.Path1);
                state = 1;
                break;

            case 1:
                if (!follower.isBusy()) beginWait(waitShoot, true, 2); // shoot at first 59.244,84.166
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
                if (!follower.isBusy()) beginWait(waitShoot, true, 5); // shoot at 59.244,84.166 again
                break;

            case 5:
                if (waitDone()) {
                    follower.followPath(paths.Path4);
                    state = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) beginWait(waitGate, false, 7); // at gate, no shooting
                break;

            case 7:
                if (waitDone()) {
                    follower.followPath(paths.Path5);
                    state = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) beginWait(waitShoot, true, 9); // shoot again at 59.244,84.166
                break;

            case 9:
                if (waitDone()) {
                    follower.followPath(paths.Path6);
                    state = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) beginWait(waitGate, false, 11); // second gate wait
                break;

            case 11:
                if (waitDone()) {
                    follower.followPath(paths.Path7);
                    state = 12;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    state = 13;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    state = 14;
                }
                break;

            case 14:
                if (!follower.isBusy()) beginWait(waitShoot, true, 15); // shoot at 59.244,84.166 again
                break;

            case 15:
                if (waitDone()) {
                    follower.followPath(paths.Path10);
                    state = 16;
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11);
                    state = 17;
                }
                break;

            case 17:
                if (!follower.isBusy()) beginWait(waitTiny, true, 18); // tiny final shot delay
                break;

            case 18:
                if (waitDone()) {
                    follower.followPath(paths.Path12);
                    state = 19;
                }
                break;

            case 19:
                // done
                break;
        }
    }

    private void updateHoodAndVelocity() {
        boolean active =
                state == 0 || state == 1 || state == 2 || state == 3 ||
                        state == 4 || state == 5 || state == 6 || state == 7 ||
                        state == 8 || state == 9 || state == 10 || state == 11 ||
                        state == 12 || state == 13 || state == 14 || state == 15 ||
                        state == 16 || state == 17 || state == 18;

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

        if (distance >= 110) {
            double originalPos = -0.000000217243 * Math.pow(distance, 3)
                    + 0.0000386489 * Math.pow(distance, 2)
                    + 0.00297592 * distance
                    + 0.413722;

            hoodServo.setPosition(originalPos - 0.4);

            targetVelocity = (-0.0111536 * distance * distance
                    - 4.00719 * distance
                    - 1097.37524);
        } else {
            double originalPos = -5.81745e-7 * Math.pow(distance, 3)
                    + 0.0000705013 * Math.pow(distance, 2)
                    + 0.00433215 * distance
                    + 0.212657;

            hoodServo.setPosition(originalPos - 0.4);

            targetVelocity = (-0.0590251 * Math.pow(distance, 2)
                    + 2.85266 * distance
                    - 1304.88019);
        }
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
        double maxVelocity = outtake2Motor.getMotorType().getMaxRPM()
                * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;

        double normalizedError = (targetVelocity - currentVelocity) / maxVelocity;
        double pTerm = vkP * normalizedError;

        double feedforward = vkF * (targetVelocity / maxVelocity);
        double batteryVoltage = Math.max(10.5, batteryVoltageSensor.getVoltage());
        feedforward *= 12.0 / batteryVoltage;

        double power = Math.max(-1.0, Math.min(1.0, pTerm + feedforward));

        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);

        telemetry.addData("Flywheel Target V", targetVelocity);
        telemetry.addData("Flywheel Actual V", currentVelocity);
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
    @Override public void opTeardown() {}
}

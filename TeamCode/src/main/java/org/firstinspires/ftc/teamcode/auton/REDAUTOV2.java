package org.firstinspires.ftc.teamcode.auton;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;
import org.firstinspires.ftc.teamcode.opmodes.V2RED;


@Config
@Autonomous(name = "REDAUTOV2")
public class REDAUTOV2 extends DbzOpMode {


    public static double servooffset = 0.01;
    public static double Push0 = 0.81;
    public static double Push1 = 0.67;
    public static double Push2 = 0.47;
    public static double Push3 = 0.22;


    public static double lock = 0.71;
    public static double holdOpenPos = 0.8;
    public static double holdClosePos = 0.8;
    public static double dthresh = 0.24;
    public static double detectionDebounce = 0.5;
    public static double intakeWaitTimeout = 0.35;


    public static double TV = 0;
    public static double hoodServoPos = 0.5;
    public static double vkP = 5, vkF = 1.2, vkD = 0.0, vkDMax = 0.25;


    public static double velA = -0.0157003, velB = 11.6092, velC = 727.08688;
    public static double hoodA = -0.0000876693, hoodB = 0.0228448, hoodC = -0.779915;
    public static double timeA = 0.00002, timeB = 0.004, timeC = 0.25;


    public static double goalx = 143, goaly = 140;


    public static double hoodDipDuringShot = 0.015;
    public static double dipDelaySec = 0.5;
    public static double dipDurationSec = 0.15;


    public static double turretZeroDeg = 323;
    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.001;
    public static double turretDeadbandDeg = 0.0;
    public static double turretMaxPower = 1.0;
    public static double turretKs = 0.0;
    public static double turretFFDeadbandDeg = 0.0;
    public static double turretHeadingOffsetDeg = 0.0;


    public static double startX = 114.2417;
    public static double startY = 133.472;
    public static double gatex = 145.1;
    public static double gatey = 59.85;


    public static double gateh = 24;


    //135.35, 58.39, 33.1






    public static double shootX = 88.149;
    public static double shootY = 86.168;


    public static double wall1X = 127.544;
    public static double wall1Y = 59.850;
    public static double wall1Cx = 99.885;
    public static double wall1CxBack = 99.704;
    public static double wall1Cy = 54.734;


    public static double gateX = 137.544;
    public static double gateY = 62.3;
    public static double gateCx = 105.885;
    public static double gateCy = 50.734;


    public static double threshold = 220;
    public static double threshold2 = 180;


    public static double nearWallX = 124.573;
    public static double nearWallY = 84.566;


    public static double lowWallX = 127.207;
    public static double lowWallY = 35.159;
    public static double lowWallCx = 89.091;
    public static double lowWallCy = 30.918;


    public static double finalX = 89.070;
    public static double finalY = 110.914;




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


        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.417, 136.815),


                                    new Pose(89.149, 84.168)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))


                    .build();


            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.149, 84.168),
                                    new Pose(99.885, 54.734),
                                    new Pose(127.544, 59.850)
                            )
                    ) .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))


                    .build();


            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(127.544, 59.850),
                                    new Pose(99.704, 54.734),
                                    new Pose(89.149, 84.168)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))


                    .build();


            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.149, 84.168),
                                    new Pose(110.990, 60.361),


                                    new Pose(gatex, gatey)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gateh))


                    .build();


            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(gatex, gatey),
                                    new Pose(110.990, 60.361),


                                    new Pose(89.149, 84.168)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(gateh), Math.toRadians(0))


                    .build();


            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.149, 84.168),
                                    new Pose(110.990, 60.361),


                                    new Pose(gatex, gatey)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gateh))


                    .build();


            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(gatex, gatey),
                                    new Pose(110.990, 60.361),
                                    new Pose(89.149, 84.168)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(gateh), Math.toRadians(0))


                    .build();


            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.149, 84.168),
                                    new Pose(110.990, 60.361),
                                    new Pose(gatex, gatey)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gateh))


                    .build();


            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(gatex, gatey),
                                    new Pose(110.990, 60.361),
                                    new Pose(89.149, 84.168)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(gateh), Math.toRadians(0))


                    .build();


            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.149, 84.168),




                                    new Pose(128.573, 84.566)
                            )
                    ).setTangentHeadingInterpolation()


                    .build();


            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.573, 84.566),


                                    new Pose(89.149, 84.168)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            Path12 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.149, 84.168),
                                    new Pose(89.091, 30.918),
                                    new Pose(127.207, 35.159)
                            )
                    ).setTangentHeadingInterpolation()


                    .build();


            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.207, 35.159),


                                    new Pose(89.070, 110.914)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()


                    .build();
        }
    }






    protected Servo rightpushServo, leftpushServo, hoodServo, holdServo;
    protected DcMotorEx intakeMotor, turret, outtake1Motor, outtake2Motor;
    private VoltageSensor batteryVoltageSensor;
    private AnalogInput turretEncoder, distancez;


    private Follower follower;
    private Paths paths;


    private enum AutonState {
        followPath1, shoot1,
        followPath2,
        followPath3, shoot3,
        followPath4, intakeWait1,
        followPath5, shoot5,
        followPath6, intakeWait2,
        followPath7, shoot7,
        followPath8, intakeWait3,
        followPath9, shoot9,
        followPath10,
        followPath11, shoot11,
        followPath12,
        followPath13, shoot13,
        done
    }
    private AutonState autonState = AutonState.followPath1;


    private enum BallState { idle, reversing, locked }
    private BallState ballState = BallState.idle;


    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime ballReverseTimer = new ElapsedTime();
    private ElapsedTime detectionTimer = new ElapsedTime();
    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime dipTimer = new ElapsedTime();


    private boolean wasDetected = false;


    private PIDController turretPID;


    private double targetVelocity = 0;
    private double baseHoodPos = hoodServoPos;
    private double lastVelErrorNorm = 0;
    private double lastVelTimeSec = 0;


    private boolean shooting = false;
    private boolean dipActive = false;
    private boolean dipDone = false;


    @Override
    public void opInit() {
        rightpushServo = hardwareMap.get(Servo.class, "rightpushServo");
        leftpushServo = hardwareMap.get(Servo.class, "leftpushServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        holdServo = hardwareMap.get(Servo.class, "holdServo");


        distancez = hardwareMap.get(AnalogInput.class, "distance");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");


        intakeMotor = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;
        outtake1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2Motor.setDirection(DcMotorEx.Direction.FORWARD);


        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        turretPID = new PIDController(turretKp, turretKi, turretKd);
        turretPID.setTolerance(1.0);


        hoodServo.setPosition(hoodServoPos);
        holdServo.setPosition(holdClosePos);
        leftpushServo.setPosition(lock);
        rightpushServo.setPosition(lock - servooffset);


        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(270)));
        paths = new Paths(follower);


        velocityTimer.reset();
        lastVelTimeSec = 0;


        follower.followPath(paths.Path1, true);
        stateTimer.reset();
    }


    @Override
    public void opLoop() {
        follower.update();


        applyHoodAndVelocityRegressions();
        runFlywheelVelocityControl();
        aim();
        dipshot();


        switch (autonState) {


            case followPath1:


                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.shoot1;
                }
                break;


            case shoot1:


                if (stateTimer.seconds() >= 0.4) {
                    endShoot();
                    follower.followPath(paths.Path2, true);
                    autonState = AutonState.followPath2;
                }
                break;


            case followPath2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    intakeMotor.setPower(1);
                    autonState = AutonState.followPath3;
                }
                break;


            case followPath3:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.shoot3;
                }
                break;


            case shoot3:
                if (stateTimer.seconds() >= 0.4) {
                    endShoot();
                    intakeMotor.setPower(1);
                    follower.followPath(paths.Path4, true);
                    autonState = AutonState.followPath4;
                }
                break;


            case followPath4:
                intakeMotor.setPower(1);
                runBallDetection();
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autonState = AutonState.intakeWait1;
                }
                break;


            case intakeWait1:
                runBallDetection();
                if (ballState == BallState.locked || stateTimer.seconds() >= 0.2) {
                    ballState = BallState.idle;
                    wasDetected = false;
                    follower.followPath(paths.Path5, true);
                    autonState = AutonState.followPath5;
                }
                break;


            case followPath5:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.shoot5;
                }
                break;


            case shoot5:
                if (stateTimer.seconds() >= 0.4) {
                    endShoot();
                    intakeMotor.setPower(1);
                    follower.followPath(paths.Path6, true);
                    autonState = AutonState.followPath6;
                }
                break;


            case followPath6:
                intakeMotor.setPower(1);
                runBallDetection();
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autonState = AutonState.intakeWait2;
                }
                break;


            case intakeWait2:
                runBallDetection();
                if (ballState == BallState.locked || stateTimer.seconds() >= intakeWaitTimeout) {
                    ballState = BallState.idle;
                    wasDetected = false;
                    follower.followPath(paths.Path7, true);
                    autonState = AutonState.followPath7;
                }
                break;


            case followPath7:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.shoot7;
                }
                break;


            case shoot7:
                if (stateTimer.seconds() >= 0.4) {
                    endShoot();
                    intakeMotor.setPower(1);
                    follower.followPath(paths.Path8, true);
                    autonState = AutonState.followPath8;
                }
                break;


            case followPath8:
                intakeMotor.setPower(1);
                runBallDetection();
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autonState = AutonState.intakeWait3;
                }
                break;


            case intakeWait3:
                runBallDetection();
                if (ballState == BallState.locked || stateTimer.seconds() >= intakeWaitTimeout) {
                    ballState = BallState.idle;
                    wasDetected = false;
                    follower.followPath(paths.Path9, true);
                    autonState = AutonState.followPath9;
                }
                break;


            case followPath9:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.shoot9;
                }
                break;


            case shoot9:
                if (stateTimer.seconds() >= 0.4) {
                    endShoot();
                    follower.followPath(paths.Path10, true);
                    autonState = AutonState.followPath10;
                }
                break;


            case followPath10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, true);
                    autonState = AutonState.followPath11;
                }
                break;


            case followPath11:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.shoot11;
                }
                break;


            case shoot11:
                if (stateTimer.seconds() >= 0.4) {
                    endShoot();
                    follower.followPath(paths.Path12, true);
                    autonState = AutonState.followPath12;
                }
                break;


            case followPath12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path13, true);
                    autonState = AutonState.followPath13;
                }
                break;


            case followPath13:
                if (!follower.isBusy()) {
                    startShoot();
                    stateTimer.reset();
                    autonState = AutonState.shoot13;
                }
                break;


            case shoot13:
                if (stateTimer.seconds() >= 0.4) {
                    endShoot();
                    autonState = AutonState.done;
                }
                break;


            case done:
                intakeMotor.setPower(0);
                outtake1Motor.setPower(0);
                outtake2Motor.setPower(0);
                turret.setPower(0);
                break;
        }


        telemetry.addData("State", autonState);
        telemetry.addData("Ball State", ballState);
        telemetry.addData("Sensor V", String.format("%.3f", distancez.getVoltage()));
        telemetry.addData("Flywheel V", String.format("%.0f", outtake2Motor.getVelocity()));
        telemetry.addData("Target V", String.format("%.0f", targetVelocity));
        telemetry.addData("Turret Deg", String.format("%.1f", getTurretAngleDeg()));
        telemetry.update();
    }


    private void runBallDetection() {
        double dist = distancez.getVoltage();
        boolean detected = dist < dthresh;


        switch (ballState) {
            case idle:
                if (detected) {
                    if (!wasDetected) {
                        detectionTimer.reset();
                        wasDetected = true;
                    }
                    if (detectionTimer.seconds() >= 0.36) {
                        holdServo.setPosition(holdClosePos);
                        leftpushServo.setPosition(lock);
                        rightpushServo.setPosition(lock - servooffset);
                        intakeMotor.setPower(-1);
                        ballReverseTimer.reset();
                        ballState = BallState.reversing;
                        wasDetected = false;
                    }
                } else {
                    wasDetected = false;
                }
                break;


            case reversing:
                holdServo.setPosition(holdClosePos);
                if (!shooting && ballReverseTimer.seconds() < 1.0) {
                    leftpushServo.setPosition(lock);
                    rightpushServo.setPosition(lock - servooffset);
                }
                intakeMotor.setPower(-1);
                if (ballReverseTimer.seconds() >= 1.0) {
                    intakeMotor.setPower(1);
                    leftpushServo.setPosition(Push0);
                    rightpushServo.setPosition(Push0 - servooffset);
                    ballState = BallState.locked;
                }
                break;


            case locked:
                if (!shooting) {
                    leftpushServo.setPosition(Push0);
                    rightpushServo.setPosition(Push0 - servooffset);
                    intakeMotor.setPower(1);
                    wasDetected = false;
                }
                break;
        }
    }


    private void startShoot() {
        holdServo.setPosition(holdOpenPos);
        leftpushServo.setPosition(Push3);
        rightpushServo.setPosition(Push3 - servooffset);
        shooting = true;
        dipActive = false;
        dipDone = false;
    }


    private void endShoot() {
        leftpushServo.setPosition(Push0);
        rightpushServo.setPosition(Push0 - servooffset);
        holdServo.setPosition(holdClosePos);
        shooting = false;
        ballState = BallState.idle;
        wasDetected = false;
    }


    private void applyHoodAndVelocityRegressions() {
        Pose poseNow = follower.getPose();
        if (poseNow == null) {
            baseHoodPos = (hoodServoPos+.009);
            targetVelocity = TV;
            return;
        }
        Pose vGoal = updateGoalV2(poseNow);
        double dist = Math.hypot(vGoal.getX() - poseNow.getX(), vGoal.getY() - poseNow.getY());


        double hoodPos = (hoodA * dist * dist) + (hoodB * dist) + hoodC;
        baseHoodPos = Math.max(0.0, Math.min(1.0, hoodPos));


        double vel = (velA * dist * dist) + (velB * dist) + velC;
        double maxVel = outtake2Motor.getMotorType().getMaxRPM()
                * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;
        targetVelocity = Math.max(-maxVel, Math.min(maxVel, vel));
    }


    private void dipshot() {
        if (shooting && !dipActive && !dipDone) {
            dipActive = true;
            dipTimer.reset();
        }
        if (!shooting) {
            dipActive = false;
            dipDone = false;
            hoodServo.setPosition(baseHoodPos);
            return;
        }
        if (dipActive) {
            double elapsed = dipTimer.seconds();
            if (elapsed < dipDelaySec) {
                hoodServo.setPosition(baseHoodPos);
            } else if (elapsed < dipDelaySec + dipDurationSec) {
                hoodServo.setPosition(Math.max(0.0, baseHoodPos - hoodDipDuringShot));
            } else {
                hoodServo.setPosition(baseHoodPos);
                dipActive = false;
                dipDone = true;
            }
        }
    }


    private Pose updateGoalV2(Pose robotPose) {
        Vector vel = follower.getVelocity();
        double vx = (vel != null) ? vel.getXComponent() : 0.0;
        double vy = (vel != null) ? vel.getYComponent() : 0.0;
        double speed = Math.hypot(vx, vy);
        if (speed < 1.5) { vx = 0; vy = 0; }
        double dist = Math.hypot(goalx - robotPose.getX(), goaly - robotPose.getY());
        double shotTime = (timeA * dist * dist) + (timeB * dist) + timeC;
        return new Pose(goalx - (vx * shotTime), goaly - (vy * shotTime), 0);
    }


    private void aim() {


        double targetAngleDeg = 0;



        double desiredClamped = overshoot();
        if (Math.abs(desiredClamped - getTurretAngleDeg()) <= threshold) {
            targetAngleDeg = desiredClamped;
        }


        double currentAngleDeg = getTurretAngleDeg();



        double errorDeg = angleWrap( targetAngleDeg - currentAngleDeg);



        if (Math.abs(errorDeg) <= turretDeadbandDeg) {
            turret.setPower(0);
            return;
        }


        turretPID.setPID(turretKp, turretKi, turretKd);
        double pidOut = turretPID.calculate(currentAngleDeg, targetAngleDeg);


        double ff = 0.0;
        if (Math.abs(errorDeg) > turretFFDeadbandDeg) {
            ff = Math.copySign(turretKs, errorDeg);
        }


        double output = pidOut + ff;


        if (output > turretMaxPower) output = turretMaxPower;
        if (output < -turretMaxPower) output = -turretMaxPower;


        turret.setPower(output);
    }
    private void runFlywheelVelocityControl() {
        if (Math.abs(targetVelocity) <= 1.0) {
            outtake1Motor.setPower(0);
            outtake2Motor.setPower(0);
            lastVelErrorNorm = 0.0;
            lastVelTimeSec = velocityTimer.seconds();
            return;
        }


        double currentVelocity = outtake2Motor.getVelocity();
        double maxVelocity = outtake2Motor.getMotorType().getMaxRPM()
                * outtake2Motor.getMotorType().getTicksPerRev() / 60.0;


        double nowSec = velocityTimer.seconds();
        double dt = Math.max(1e-3, Math.min(0.1, nowSec - lastVelTimeSec));


        double normalizedError = (targetVelocity - currentVelocity) / maxVelocity;
        double pTerm = vkP * normalizedError;
        double dErr = (normalizedError - lastVelErrorNorm) / dt;
        double dTerm = Math.max(-vkDMax, Math.min(vkDMax, vkD * dErr));


        double feedforward = vkF * (targetVelocity / maxVelocity);
        feedforward *= 12.0 / Math.max(10.5, batteryVoltageSensor.getVoltage());


        double power = Math.max(-1.0, Math.min(1.0, pTerm + dTerm + feedforward));
        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);


        lastVelErrorNorm = normalizedError;
        lastVelTimeSec = nowSec;
    }


    private double getTurretAngleDeg() {
        double voltage = turretEncoder.getVoltage();
        double angle = (voltage / turretEncoder.getMaxVoltage()) * 360.0;
        angle -= turretZeroDeg;
        return angleWrapAsym(angle, threshold);
    }


    private double getDesiredTurretAngleDeg() {
        Pose pose = follower.getPose();
        if (pose == null) return getTurretAngleDeg();


        Pose vGoal = updateGoalV2(pose);


        double fieldAngle = Math.atan2(vGoal.getY() - pose.getY(), vGoal.getX() - pose.getX());
        double turretAngleDeg = Math.toDegrees(fieldAngle - pose.getHeading()) + turretHeadingOffsetDeg;


        return angleWrapAsym(turretAngleDeg, threshold);
    }


    private double overshoot() {
        double rawAngle;
        if (autonState == AutonState.shoot1 || autonState == AutonState.followPath1 ) {
            rawAngle = 40;
        } else if (autonState == AutonState.shoot13 || (autonState == AutonState.followPath13)) {
            rawAngle = 90;
        } else {
            rawAngle = 44;
        }


        double desired = angleWrapAsym(rawAngle, threshold);
        if (desired > threshold2) return threshold2;
        if (desired < -threshold) return -threshold;
        return desired;
    }


    private double angleWrap(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }


    private double angleWrapAsym(double angle, double negLimit) {
        return ((angle + negLimit) % 360 + 360) % 360 - negLimit;
    }


    @Override public void opLoopHook() {}


    @Override
    public void opTeardown() {}
}


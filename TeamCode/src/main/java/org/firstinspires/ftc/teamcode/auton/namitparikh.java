package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@Autonomous(name = "namitparikh", group = "Autonomous")
public class namitparikh extends DbzOpMode {

    private Follower driveFollower;
    private Paths autoPaths;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();

    private int autoState = 0;

    private DcMotorEx intakeMotor;
    private DcMotorEx leftFlywheelMotor;
    private DcMotorEx rightFlywheelMotor;

    private Servo leftPusherServo;
    private Servo rightPusherServo;
    private Servo hoodServo;

    private VoltageSensor batteryVoltageSensor;

    // Servo positions
    public static double leftPusherShootPos = 0.90;
    public static double rightPusherShootPos = 0.86;
    public static double leftPusherIdlePos = 0.25;
    public static double rightPusherIdlePos = 0.21;
    public static double hoodIdlePos = 0.80;

    // Flywheel control
    public static double flywheelTargetVelocity = -1400;
    public static double flywheelKp = 2.742;
    public static double flywheelKf = 1.27;

    private boolean isShooting = false;

    // Bucket position (example — adjust to actual field coordinates)
    private static double bucketX = 0.0;
    private static double bucketY = 0.0;

    @Override
    protected void opInit() {
        intakeMotor = robot.intakeMotor;
        leftFlywheelMotor = robot.outtake1Motor;
        rightFlywheelMotor = robot.outtake2Motor;

        leftPusherServo = hardwareMap.get(Servo.class, "leftpushServo");
        rightPusherServo = hardwareMap.get(Servo.class, "rightpushServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        rightFlywheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);

        hoodServo.setPosition(hoodIdlePos);
        leftPusherServo.setPosition(leftPusherIdlePos);
        rightPusherServo.setPosition(rightPusherIdlePos);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        driveFollower = Constants.createFollower(hardwareMap);
        driveFollower.setStartingPose(new Pose(21.084, 123.813, Math.toRadians(142)));

        autoPaths = new Paths(driveFollower);

        autoState = 0;
        stateTimer.reset();
    }

    @Override
    protected void opLoopHook() {

    }

    @Override
    protected void opLoop() {
        // Update follower and flywheel
        driveFollower.update();
        runFlywheelVelocityControl();
        updateShotSequence();

        switch (autoState) {
            case 0:
                intakeMotor.setPower(1);
                if (startPath(autoPaths.toBallsFromBin)) autoState = 1;
                break;

            case 1:
                intakeMotor.setPower(1);
                if (startPath(autoPaths.ballsToLeverAndBack)) autoState = 2;
                break;

            case 2:
                intakeMotor.setPower(1);
                if (startPath(autoPaths.sweepBallsMidLane)) autoState = 3;
                break;

            case 3:
                intakeMotor.setPower(1);
                if (startPath(autoPaths.returnToBinFromBalls)) autoState = 4;
                break;

            case 4:
                intakeMotor.setPower(0);

                // Face bucket before shooting
                faceBucket();

                if (!isShooting) startShotOneSecond();

                if (!isShooting && stateTimer.seconds() > 0.5) {
                    driveFollower.followPath(autoPaths.binToBallsLowLane, true);
                    stateTimer.reset();
                    autoState = 5;
                }
                break;

            case 5:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    driveFollower.followPath(autoPaths.ballsLowLaneBackToBin, true);
                    stateTimer.reset();
                    autoState = 6;
                }
                break;

            case 6:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    stateTimer.reset();
                    autoState = 7;
                }
                break;

            case 7:
                intakeMotor.setPower(0);
                faceBucket();
                if (!isShooting) startShotOneSecond();
                if (!isShooting && stateTimer.seconds() > 0.5) {
                    driveFollower.followPath(autoPaths.binToBallsLowLaneAgain, true);
                    stateTimer.reset();
                    autoState = 8;
                }
                break;

            case 8:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    driveFollower.followPath(autoPaths.ballsLowLaneAgainBackToBin, true);
                    stateTimer.reset();
                    autoState = 9;
                }
                break;

            case 9:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    stateTimer.reset();
                    autoState = 10;
                }
                break;

            case 10:
                intakeMotor.setPower(0);
                faceBucket();
                if (!isShooting) startShotOneSecond();
                if (!isShooting && stateTimer.seconds() > 0.5) {
                    driveFollower.followPath(autoPaths.binToBallsUpperLaneCurve, true);
                    stateTimer.reset();
                    autoState = 11;
                }
                break;

            case 11:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    driveFollower.followPath(autoPaths.sweepBallsUpperLane, true);
                    stateTimer.reset();
                    autoState = 12;
                }
                break;

            case 12:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    driveFollower.followPath(autoPaths.returnToBinFromUpperLane, true);
                    stateTimer.reset();
                    autoState = 13;
                }
                break;

            case 13:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    stateTimer.reset();
                    autoState = 14;
                }
                break;

            case 14:
                intakeMotor.setPower(0);
                faceBucket();
                if (!isShooting) startShotOneSecond();
                if (!isShooting && stateTimer.seconds() > 0.5) {
                    driveFollower.followPath(autoPaths.binToBallsLowerLaneCurve, true);
                    stateTimer.reset();
                    autoState = 15;
                }
                break;

            case 15:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    driveFollower.followPath(autoPaths.sweepBallsLowerLane, true);
                    stateTimer.reset();
                    autoState = 16;
                }
                break;

            case 16:
                intakeMotor.setPower(1);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.2) {
                    driveFollower.followPath(autoPaths.returnToBinFromLowerLane, true);
                    stateTimer.reset();
                    autoState = 17;
                }
                break;

            case 17:
                intakeMotor.setPower(0);
                faceBucket();
                if (!isShooting) startShotOneSecond();
                if (!isShooting && stateTimer.seconds() > 0.5) {
                    driveFollower.followPath(autoPaths.parkLeftFromBin, true);
                    stateTimer.reset();
                    autoState = 18;
                }
                break;

            case 18:
                intakeMotor.setPower(0);
                if (!driveFollower.isBusy() && stateTimer.seconds() > 0.5) {
                    leftFlywheelMotor.setPower(0);
                    rightFlywheelMotor.setPower(0);
                    autoState = -1;
                }
                break;
        }
    }

    @Override
    protected void opTeardown() { }

    private boolean startPath(PathChain path) {
        if (!driveFollower.isBusy() && stateTimer.seconds() > 0.5) {
            driveFollower.followPath(path, true);
            stateTimer.reset();
            return true;
        }
        return false;
    }

    private void startShotOneSecond() {
        isShooting = true;
        shotTimer.reset();
        stateTimer.reset();
        leftPusherServo.setPosition(leftPusherShootPos);
        rightPusherServo.setPosition(rightPusherShootPos);
    }

    private void updateShotSequence() {
        if (isShooting && shotTimer.milliseconds() >= 1000) {
            leftPusherServo.setPosition(leftPusherIdlePos);
            rightPusherServo.setPosition(rightPusherIdlePos);
            isShooting = false;
            stateTimer.reset();
        }
    }

    private void runFlywheelVelocityControl() {
        double currentVelocity = rightFlywheelMotor.getVelocity();
        double maxVelocity = rightFlywheelMotor.getMotorType().getMaxRPM()
                * rightFlywheelMotor.getMotorType().getTicksPerRev() / 60.0;

        double normalizedError = (flywheelTargetVelocity - currentVelocity) / maxVelocity;
        double pidTerm = flywheelKp * normalizedError;

        double feedforwardTerm = flywheelKf * (flywheelTargetVelocity / maxVelocity);
        double batteryVoltage = Math.max(10.5, batteryVoltageSensor.getVoltage());
        feedforwardTerm *= 12.0 / batteryVoltage;

        double motorPower = Math.max(-1, Math.min(1, pidTerm + feedforwardTerm));

        leftFlywheelMotor.setPower(motorPower);
        rightFlywheelMotor.setPower(motorPower);
    }

    // --- Face the bucket (simple proportional turn) ---
    private void faceBucket() {
        Pose pose = driveFollower.getPose();
        if (pose == null) return;

        double dx = bucketX - pose.getX();
        double dy = bucketY - pose.getY();
        double targetHeading = Math.atan2(dy, dx); // radians

        double error = angleWrapRad(targetHeading - pose.getHeading());

        double turnPower = 0.8 * error;
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        driveFollower.setTeleOpDrive(0, 0, turnPower, true);
    }

    private double angleWrapRad(double angle) {
        return ((angle + Math.PI) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI) - Math.PI;
    }

    // --- Path definitions ---
    public static class Paths {
        public PathChain toBallsFromBin;
        public PathChain ballsToLeverAndBack;
        public PathChain sweepBallsMidLane;
        public PathChain returnToBinFromBalls;

        public PathChain binToBallsLowLane;
        public PathChain ballsLowLaneBackToBin;

        public PathChain binToBallsLowLaneAgain;
        public PathChain ballsLowLaneAgainBackToBin;

        public PathChain binToBallsUpperLaneCurve;
        public PathChain sweepBallsUpperLane;
        public PathChain returnToBinFromUpperLane;

        public PathChain binToBallsLowerLaneCurve;
        public PathChain sweepBallsLowerLane;
        public PathChain returnToBinFromLowerLane;

        public PathChain parkLeftFromBin;

        public Paths(Follower f) {
            // Define all paths exactly as before
            toBallsFromBin = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.866, 123.392), new Pose(48.333, 95.669)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                    .build();

            ballsToLeverAndBack = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(48.333, 95.669),
                            new Pose(82.344, 59.271),
                            new Pose(41.172, 59.867)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            sweepBallsMidLane = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(41.172, 59.867), new Pose(15.316, 59.867)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            returnToBinFromBalls = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(15.316, 59.867),
                            new Pose(79.360, 65.635),
                            new Pose(48.333, 95.867)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            binToBallsLowLane = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(48.333, 95.867),
                            new Pose(51.913, 37.790),
                            new Pose(11.736, 62.456)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();

            ballsLowLaneBackToBin = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(11.736, 62.456),
                            new Pose(79.360, 65.635),
                            new Pose(48.333, 95.867)))
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))
                    .build();

            binToBallsLowLaneAgain = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(48.333, 95.867),
                            new Pose(51.913, 37.790),
                            new Pose(11.935, 62.456)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))
                    .build();

            ballsLowLaneAgainBackToBin = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(11.935, 62.456),
                            new Pose(79.360, 79.360),
                            new Pose(48.333, 95.867)))
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))
                    .build();

            binToBallsUpperLaneCurve = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(48.333, 95.867),
                            new Pose(57.681, 85.127),
                            new Pose(41.172, 83.735)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            sweepBallsUpperLane = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(41.172, 83.735), new Pose(16.310, 83.934)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            returnToBinFromUpperLane = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(16.310, 83.934), new Pose(48.333, 95.867)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            binToBallsLowerLaneCurve = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(48.333, 95.867),
                            new Pose(70.609, 58.077),
                            new Pose(42.167, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            sweepBallsLowerLane = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(42.167, 36.000), new Pose(16.509, 35.801)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            returnToBinFromLowerLane = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(16.509, 35.801), new Pose(48.333, 95.867)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            parkLeftFromBin = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.333, 95.867), new Pose(15.913, 95.867)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }
}

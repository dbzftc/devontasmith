package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;
import org.firstinspires.ftc.teamcode.auton.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@Autonomous(name = "namitparikh", group = "Autonomous")
public class namitparikh extends DbzOpMode {

    private Follower follower;
    private ElapsedTime pathTimer = new ElapsedTime();
    private int pathState = 0;
    private Paths paths;
    private PIDController controller;

    protected Servo holdServo, shoot1Servo, shoot2Servo, pushServo;
    private VoltageSensor batteryVoltageSensor;

    protected DcMotorEx intakeMotor, outtake1Motor, outtake2Motor;
    public static double kP = 0.00014;
    public static double kI = 0.0;
    public static double kD = 0.000012;
    public static double kF = 0.00043;
    public static double targetVelocity = -1700;
    public static double holdPos = 0.12;
    public static double holdPos2 = 0.3;
    public static double shootPos = 0.0;

    @Override
    protected void opInit() {
        intakeMotor = robot.intakeMotor;
        outtake1Motor = robot.outtake1Motor;
        outtake2Motor = robot.outtake2Motor;
        robot.holdServo.setPosition(0.2);

        holdServo = hardwareMap.get(Servo.class, "holdServo");
        shoot1Servo = hardwareMap.get(Servo.class, "shoot1Servo");
        shoot2Servo = hardwareMap.get(Servo.class, "shoot2Servo");

        pushServo = hardwareMap.get(Servo.class, "pushServo");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.084, 123.813, Math.toRadians(142)));
        paths = new Paths(follower);
        robot.holdServo.setPosition(holdPos);

        controller = new PIDController(kP, kI, kD);
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        pathState = 0;
    }

    @Override
    protected void opLoop() {

        follower.update();
        controller.setPID(kP, kI, kD);
        outtake2Motor.setDirection(DcMotorEx.Direction.FORWARD);
        outtake1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        controller.setIntegrationBounds(-0.3, 0.3);

        double currentVelocity = outtake2Motor.getVelocity();
        double pid = controller.calculate(currentVelocity, targetVelocity);

        double batteryVoltage = batteryVoltageSensor.getVoltage();
        double feedforward = (kF * targetVelocity) * (12.0 / batteryVoltage);

        double power = pid + feedforward;
        power = Math.max(-1, Math.min(1, power));

        intakeMotor.setPower(-1);
        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);

        switch (pathState) {
            case 0:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path1, true);
                    pathTimer.reset();
                    pathState = 1;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    robot.holdServo.setPosition(holdPos2);
                    robot.pushServo.setPosition(0.75);
                    if (pathTimer.seconds() > 5.0) {
                        robot.pushServo.setPosition(0.2);
                        robot.holdServo.setPosition(holdPos);
                        follower.followPath(paths.Path2, true);
                        pathTimer.reset();
                        pathState = 2;
                    }
                }
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path3, true);
                    pathTimer.reset();
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path4, true);
                    pathTimer.reset();
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    robot.holdServo.setPosition(holdPos2);
                    robot.pushServo.setPosition(0.75);
                    if (pathTimer.seconds() > 5.0) {
                        robot.pushServo.setPosition(0.2);
                        robot.holdServo.setPosition(holdPos);
                        follower.followPath(paths.Path5, true);
                        pathTimer.reset();
                        pathState = 5;
                    }
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.seconds() > 2) {
                    follower.followPath(paths.Path6, true);
                    pathTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    robot.holdServo.setPosition(holdPos2);
                    robot.pushServo.setPosition(0.75);
                    if (pathTimer.seconds() > 5.0) {
                        robot.pushServo.setPosition(0.2);
                        robot.holdServo.setPosition(holdPos);
                        follower.followPath(paths.Path7, true);
                        pathTimer.reset();
                        pathState = 7;
                    }
                }
                break;

            case 7:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path8, true);
                    pathTimer.reset();
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy() && pathTimer.seconds() > 2) {
                    follower.followPath(paths.Path9, true);
                    pathTimer.reset();
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path10, true);
                    pathTimer.reset();
                    pathState = 10; n
                }
                break;

            case 10:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path11, true);
                    pathTimer.reset();
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path12, true);
                    pathTimer.reset();
                    pathState = 12;
                }
                break;

            case 12:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path13, true);
                    pathTimer.reset();
                    pathState = 13;
                }
                break;

            case 13:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path14, true);
                    pathTimer.reset();
                    pathState = 14;
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    robot.holdServo.setPosition(holdPos2);
                    robot.pushServo.setPosition(0.75);
                    if (pathTimer.seconds() > 2.0) {
                        robot.pushServo.setPosition(0.2);
                        robot.holdServo.setPosition(holdPos);
                        follower.followPath(paths.Path15, true);
                        pathTimer.reset();
                        pathState = 15;
                    }
                }
                break;

            case 15:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    pathState = -1;
                }
                break;
        }



    }

    @Override
    protected void opTeardown() {}

    @Override
    protected void opLoopHook() {}

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
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.866, 123.392), new Pose(48.333, 95.669))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.333, 95.669),
                                    new Pose(82.344, 59.271),
                                    new Pose(41.172, 59.867)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.172, 59.867), new Pose(15.316, 59.867))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.316, 59.867),
                                    new Pose(79.360, 65.635),
                                    new Pose(48.333, 95.867)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.333, 95.867),
                                    new Pose(51.913, 37.790),
                                    new Pose(11.736, 62.456)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(135)
                    )
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.736, 65.456),
                                    new Pose(79.360, 65.635),
                                    new Pose(48.333, 95.867)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(135))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.333, 95.867),
                                    new Pose(51.913, 37.790),
                                    new Pose(11.935, 65.456)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(155))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.935, 65.456),
                                    new Pose(79.360, 79.360),
                                    new Pose(48.333, 95.867)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(135))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.333, 95.867),
                                    new Pose(57.681, 85.127),
                                    new Pose(41.172, 83.735)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.172, 83.735), new Pose(16.310, 83.934))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.310, 83.934), new Pose(48.333, 95.867))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.333, 95.867),
                                    new Pose(70.609, 58.077),
                                    new Pose(42.167, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path13 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.167, 36.000), new Pose(16.509, 35.801))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path14 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.509, 35.801), new Pose(48.333, 95.867))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path15 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.333, 95.867), new Pose(15.913, 95.867))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}


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
@Autonomous(name = "jhurts", group = "Autonomous")
public class jhurts extends DbzOpMode {

    private Follower follower;
    private ElapsedTime pathTimer = new ElapsedTime();
    private int pathState = 0;
    private Paths paths;
    private PIDController controller;

    protected Servo holdServo, shoot1Servo, shoot2Servo;
    private VoltageSensor batteryVoltageSensor;

    protected DcMotorEx intakeMotor, outtake1Motor, outtake2Motor;
    public static double kP = 0.00014;
    public static double kI = 0.0;
    public static double kD = 0.000012;
    public static double kF = 0.00043;
    public static double targetVelocity = -1700;
    public static double holdPos = 0.3;
    public static double holdPos2 = 0.12;
    public static double shootPos = 0.0;

    @Override
    protected void opInit() {
        intakeMotor = robot.intakeMotor;
//        outtake1Motor = robot.outtake1Motor;
//        outtake2Motor = robot.outtake2Motor;

        holdServo = hardwareMap.get(Servo.class, "holdServo");
        shoot1Servo = hardwareMap.get(Servo.class, "shoot1Servo");
        shoot2Servo = hardwareMap.get(Servo.class, "shoot2Servo");

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
                if (!follower.isBusy() && pathTimer.seconds() > 2) {

                    follower.followPath(paths.Path1, true);
                    pathTimer.reset();
                    pathState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path2, true);
                    pathTimer.reset();
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.seconds() > 3) {
                    robot.holdServo.setPosition(holdPos2);
                    if (pathTimer.seconds() > 6) {
                        robot.holdServo.setPosition(holdPos);
                        follower.followPath(paths.Path3, true);
                        pathTimer.reset();
                        pathState = 3;
                    }
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
                if (!follower.isBusy() && pathTimer.seconds() > 3) {
                    robot.holdServo.setPosition(holdPos2);  // open to release
                    if (pathTimer.seconds() > 6) {
                        robot.holdServo.setPosition(holdPos);  // close before starting next
                        follower.followPath(paths.Path5, true);
                        pathTimer.reset();
                        pathState = 5;
                    }
                }
                break;

            case 5:
                // Path 5: keep servo closed the entire time
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path6, true);
                    pathTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                // Wait until Path6 finishes, then open
                if (!follower.isBusy() && pathTimer.seconds() > 3) {
                    robot.holdServo.setPosition(holdPos2);  // open after path6 ends
                    if (pathTimer.seconds() > 6) {
                        robot.holdServo.setPosition(holdPos); // optionally close again if needed
                        pathState = -1;  // done
                    }
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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(20.866, 123.392),
                            new Pose(67.234, 77.796),
                            new Pose(13.395, 84.236)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13.395, 84.236),
                            new Pose(37.095, 104.844)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(37.095, 104.844),
                            new Pose(73.159, 53.839),
                            new Pose(12.880, 58.991)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.880, 58.991),
                            new Pose(49.975, 92.995)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(49.975, 92.995),
                            new Pose(84.236, 29.367),
                            new Pose(11.592, 36.322)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.592, 36.322),
                            new Pose(61.309, 13.653)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(124))
                    .build();
        }
    }
}

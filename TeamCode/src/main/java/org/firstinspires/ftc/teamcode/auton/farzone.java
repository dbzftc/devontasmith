package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.util.PoseCache;

@Config
@Autonomous(name = "farzone", group = "Autonomous")
public class farzone extends DbzOpMode {

    public static double startX = 42.000;
    public static double startY = 8.500;
    public static double startHeadingDeg = 0; // Tangent usually starts at 0 or path direction

    public static double targetX = 0.0;
    public static double targetY = 144.0;

    // CONFIGURABLE SHOOT TIME
    public static double waitShootTime = 6000;

    public static double targetVelocity = -2000;
    public static double hoodServoPos = 0.6;

    public static double holdOpenPos = 0.3;
    public static double holdClosePos = 0.06;
    public static double leftPushShoot = 0.66;
    public static double rightPushShoot = 0.69;
    public static double leftPushIdle = 0.06;
    public static double rightPushIdle = 0.09;

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
    private int state = 0;

    public static class Paths {
        public PathChain Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 8.500), new Pose(9.000, 8.500))).setTangentHeadingInterpolation().build();
            Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(9.000, 8.500), new Pose(42.000, 8.500))).setTangentHeadingInterpolation().setReversed().build();
            Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 8.500), new Pose(9.000, 8.500))).setTangentHeadingInterpolation().build();
            Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(9.000, 8.500), new Pose(42.000, 8.500))).setTangentHeadingInterpolation().setReversed().build();
            Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 8.500), new Pose(9.000, 8.500))).setTangentHeadingInterpolation().build();
            Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(9.000, 8.500), new Pose(35.000, 8.500))).setTangentHeadingInterpolation().setReversed().build();
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
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        turretPID = new PIDController(0.014, 0, 0.001);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        paths = new Paths(follower);

        hoodServo.setPosition(hoodServoPos);
    }

    @Override
    protected void opLoopHook() {

    }

    @Override
    public void opLoop() {
        follower.update();
        runFlywheelVelocityControl();
        runTurretAlwaysOn();

        holdServo.setPosition(shooting ? holdOpenPos : holdClosePos);
        leftpushServo.setPosition(shooting ? leftPushShoot : leftPushIdle);
        rightpushServo.setPosition(shooting ? rightPushShoot : rightPushIdle);

        switch (state) {
            case 0: // Initial Shoot
                beginWait(waitShootTime, true, 1);
                break;
            case 1: // Move Forward (Path 2)
                if (waitDone()) { follower.followPath(paths.Path2); state = 2; }
                break;
            case 2: // Move Back (Path 3)
                if (!follower.isBusy()) { follower.followPath(paths.Path3); state = 3; }
                break;
            case 3: // Shoot 2
                if (!follower.isBusy()) beginWait(waitShootTime, true, 4);
                break;
            case 4: // Move Forward (Path 4)
                if (waitDone()) { follower.followPath(paths.Path4); state = 5; }
                break;
            case 5: // Move Back (Path 5)
                if (!follower.isBusy()) { follower.followPath(paths.Path5); state = 6; }
                break;
            case 6: // Shoot 3
                if (!follower.isBusy()) beginWait(waitShootTime, true, 7);
                break;
            case 7: // Move Forward (Path 6)
                if (waitDone()) { follower.followPath(paths.Path6); state = 8; }
                break;
            case 8: // Move Back (Path 7)
                if (!follower.isBusy()) { follower.followPath(paths.Path7); state = 9; }
                break;
            case 9: // Done
                break;
        }
    }

    private void beginWait(double ms, boolean doshoot, int nextState) {
        waitms = ms; shooting = doshoot; waitTimer.reset(); state = nextState;
    }

    private boolean waitDone() {
        if (waitTimer.milliseconds() >= waitms) { shooting = false; return true; }
        return false;
    }

    private void runFlywheelVelocityControl() {
        double currentVelocity = outtake2Motor.getVelocity();
        double maxVelocity = 2800; // Adjust based on motor
        double power = (targetVelocity / maxVelocity) * (12.0 / batteryVoltageSensor.getVoltage());
        outtake1Motor.setPower(power);
        outtake2Motor.setPower(power);
    }

    private void runTurretAlwaysOn() {
        Pose pose = follower.getPose();
        if (pose == null) return;
        double desiredDeg = Math.toDegrees(Math.atan2(targetY - pose.getY(), targetX - pose.getX()) - pose.getHeading());
        double out = turretPID.calculate(getTurretAngleDeg(), angleWrap(desiredDeg));
        turret.setPower(Math.max(-0.3, Math.min(0.3, out)));
    }

    private double getTurretAngleDeg() {
        return angleWrap((turretEncoder.getVoltage() / turretEncoder.getMaxVoltage() * 360.0) - 295);
    }

    private double angleWrap(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    @Override
    public void opTeardown() {

    }
}
package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;

@Config
@TeleOp(name = "TurretTest")
public class TurretTest extends OpMode {
    public static double targetX = 50;
    public static double targetY = 0;
    private boolean aimingActive = false;
    private boolean lastLeftBumper = false;
    public static double kP = 0.00014;
    public static double kI = 0.0;
    public static double kD = 0.000012;
    public static double kF = 0.0;  // feedforward coefficient (ticks/sec -> power)
    private PIDController controller;

    public static Follower follower;
    private AnalogInput turretEncoder;
    protected DcMotorEx turret;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
            turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        }

        follower.setStartingPose(new Pose());

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        controller = new PIDController(kP, kI, kD);
    }


    @Override
    public void init_loop() {
        telemetryM.update(telemetry);

        follower.update();
        if (follower.getCurrentPath() != null) {
            drawOnlyCurrent();
        }


    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        // Drive robot manually
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        follower.update();
        double voltage = turretEncoder.getVoltage();
        double maxVoltage = turretEncoder.getMaxVoltage();
        double normalized = voltage / maxVoltage;
        double angleDegrees = normalized * 360;
        // Pose data for turret aiming
        telemetryM.debug("X (in): " + follower.getPose().getX());
        telemetryM.debug("Y (in): " + follower.getPose().getY());
        telemetryM.debug("Heading (rad): " + follower.getPose().getHeading());
        telemetryM.debug("Heading (deg): " + Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.debug("Turret Encoder Voltage: " + voltage);
        telemetryM.debug("Turret Angle (deg): " + angleDegrees);

        telemetryM.update(telemetry);

        if (follower.getCurrentPath() != null) {
            draw();
        }

        aim();
    }



    private void aim(){

        boolean leftBumper = gamepad1.left_bumper;
        if (leftBumper && !lastLeftBumper) {
            aimingActive = !aimingActive;
        }
        lastLeftBumper = leftBumper;

        if (!aimingActive) {
            turret.setPower(0);
        }

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getPose().getHeading(); // radians
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double angleToTarget = Math.atan2(dy, dx); // radians
        double angleToTargetDeg = Math.toDegrees(angleToTarget);
        double turretVoltage = turretEncoder.getVoltage();
        double maxVoltage = turretEncoder.getMaxVoltage();
        double normalized = turretVoltage / maxVoltage;
        double turretAngleDeg = normalized * 360;

        double zeroNormalized = 0.9327272727272727;
        double zeroDegrees = zeroNormalized * 360;
        turretAngleDeg -= zeroDegrees;

        turretAngleDeg = ((turretAngleDeg + 180) % 360) - 180;
        double desiredTurretDeg = angleToTargetDeg - Math.toDegrees(follower.getPose().getHeading());

        desiredTurretDeg = ((desiredTurretDeg + 180) % 360) - 180;


        controller.setPID(kP, kI, kD);
        controller.setIntegrationBounds(-0.3,0.3);
        double currentVoltage = turretEncoder.getVoltage();
        double targetVoltage = 3.078 + (desiredTurretDeg / 360.0) * maxVoltage;
        double pid = controller.calculate(currentVoltage, targetVoltage);

        pid = Math.max(-1, Math.min(1, pid)); // clamp
        turret.setPower(pid + kF);
    }
}
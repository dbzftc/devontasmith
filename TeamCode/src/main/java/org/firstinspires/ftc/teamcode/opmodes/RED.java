package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;


@TeleOp(name = "RED")
public class RED extends DbzOpMode {

    public static Follower follower;

    @Override
    public void opInit() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        follower.update();
        if (follower.getCurrentPath() != null) {
            drawOnlyCurrent();
        }

        follower.startTeleopDrive();
    }

    @Override
    public void opLoop() {

        follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                true
        );

        follower.update();
        telemetry.update();
    }

    @Override
    public void opLoopHook() {}

    @Override
    public void opTeardown() {}
}
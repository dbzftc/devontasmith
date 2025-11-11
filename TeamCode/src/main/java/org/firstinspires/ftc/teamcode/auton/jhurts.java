package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;
import org.firstinspires.ftc.teamcode.auton.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "jhurts", group = "Autonomous")
public class jhurts extends DbzOpMode {

    private Follower follower;
    private ElapsedTime pathTimer = new ElapsedTime();
    private int pathState = 0;
    private Paths paths;

    @Override
    protected void opInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.084, 123.813, Math.toRadians(142)));
        paths = new Paths(follower);
        pathState = 0;
    }

    @Override
    protected void opLoop() {
        follower.update();

        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, true);
                pathTimer.reset();
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    follower.followPath(paths.Path2, true);
                    pathTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.seconds() > 0.5) {
                    pathState = 3;
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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(21.084, 123.813),
                                    new Pose(48.449, 64.598),
                                    new Pose(5.159, 69.533)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(

                                    new Pose(6.729, 69.533),
                                    new Pose(37.682, 74.692),
                                    new Pose(51.140, 91.065)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();
        }
    }
}

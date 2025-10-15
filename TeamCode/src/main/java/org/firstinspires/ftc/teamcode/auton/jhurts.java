package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Autonomous(name="jhurts", group="Autonomous")
public abstract class jhurts extends DbzOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx flywheelMotor, transfer1Motor, transfer2Motor, intakeMotor;
    private Limelight3A limelight;

    private Follower follower;
    private ElapsedTime pathTimer = new ElapsedTime();

    private int pathState = 0;
    private int shotCount = 0;
    private int tagID = -1;

    private final Pose startPose = new Pose(67, 67, Math.toRadians(67));
    private final Pose shootPose1 = new Pose(67, 67, Math.toRadians(67));
    private final Pose intakePose1 = new Pose(67, 67, Math.toRadians(67));
    private final Pose shootPose2 = new Pose(67, 67, Math.toRadians(67));
    private final Pose intakePose2 = new Pose(67, 67, Math.toRadians(67));
    private final Pose shootPose3 = new Pose(67, 67, Math.toRadians(67));
    private final Pose finalPose = new Pose(67, 67, Math.toRadians(67));

    private final Pose BstartPose = new Pose(67, 67, Math.toRadians(67));
    private final Pose BshootPose1 = new Pose(67, 67, Math.toRadians(67));
    private final Pose BintakePose1 = new Pose(67, 67, Math.toRadians(67));
    private final Pose BshootPose2 = new Pose(67, 67, Math.toRadians(67));
    private final Pose BintakePose2 = new Pose(67, 67, Math.toRadians(67));
    private final Pose BshootPose3 = new Pose(67, 67, Math.toRadians(67));
    private final Pose BfinalPose = new Pose(67, 67, Math.toRadians(67));

    private final Pose CstartPose = new Pose(67, 67, Math.toRadians(67));
    private final Pose CshootPose1 = new Pose(67, 67, Math.toRadians(67));
    private final Pose CintakePose1 = new Pose(67, 67, Math.toRadians(67));
    private final Pose CshootPose2 = new Pose(67, 67, Math.toRadians(67));
    private final Pose CintakePose2 = new Pose(67, 67, Math.toRadians(67));
    private final Pose CshootPose3 = new Pose(67, 67, Math.toRadians(67));
    private final Pose CfinalPose = new Pose(67, 67, Math.toRadians(67));



    private PathChain pathToShoot1, pathToIntake1, pathToShoot2, pathToIntake2, pathToShoot3, pathToFinal;
    private PathChain BpathToShoot1, BpathToIntake1, BpathToShoot2, BpathToIntake2, BpathToShoot3, BpathToFinal;
    private PathChain CpathToShoot1, CpathToIntake1, CpathToShoot2, CpathToIntake2, CpathToShoot3, CpathToFinal;


    private final double flywheelPower = 0.85;
    private final double hoodAngle = 58.61;


    @Override
    protected void opInit() {
        frontLeft = robot.frontLeft;
        frontRight = robot.frontRight;
        backLeft = robot.backLeft;
        backRight = robot.backRight;
        flywheelMotor = robot.flywheelMotor;
        transfer1Motor = robot.transfer1Motor;
        transfer2Motor = robot.transfer2Motor;
        intakeMotor = robot.intakeMotor;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        pathState = 0;
        int seen = getID();
        if (seen == 21 || seen == 22 || seen == 23) {
            tagID=seen;
        }


    }

    private void buildPaths21() {
        pathToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        pathToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, intakePose1))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), intakePose1.getHeading())
                .build();

        pathToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, shootPose2))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), shootPose2.getHeading())
                .build();

        pathToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, intakePose2))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), intakePose2.getHeading())
                .build();

        pathToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, shootPose3))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), shootPose3.getHeading())
                .build();

        pathToFinal = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, finalPose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), finalPose.getHeading())
                .build();
    }
    private void buildPaths22() {
        BpathToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(BstartPose, BshootPose1))
                .setLinearHeadingInterpolation(BstartPose.getHeading(), BshootPose1.getHeading())
                .build();

        BpathToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(BshootPose1, BintakePose1))
                .setLinearHeadingInterpolation(BshootPose1.getHeading(), BintakePose1.getHeading())
                .build();

        BpathToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(BintakePose1, BshootPose2))
                .setLinearHeadingInterpolation(BintakePose1.getHeading(), BshootPose2.getHeading())
                .build();

        BpathToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(BshootPose2, BintakePose2))
                .setLinearHeadingInterpolation(BshootPose2.getHeading(), BintakePose2.getHeading())
                .build();

        BpathToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(BintakePose2, BshootPose3))
                .setLinearHeadingInterpolation(BintakePose2.getHeading(), BshootPose3.getHeading())
                .build();

        BpathToFinal = follower.pathBuilder()
                .addPath(new BezierLine(BshootPose3, BfinalPose))
                .setLinearHeadingInterpolation(BshootPose3.getHeading(), BfinalPose.getHeading())
                .build();
    }
    private void buildPaths23() {
        CpathToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(CstartPose, CshootPose1))
                .setLinearHeadingInterpolation(CstartPose.getHeading(), CshootPose1.getHeading())
                .build();

        CpathToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(CshootPose1, CintakePose1))
                .setLinearHeadingInterpolation(CshootPose1.getHeading(), CintakePose1.getHeading())
                .build();

        CpathToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(CintakePose1, CshootPose2))
                .setLinearHeadingInterpolation(CintakePose1.getHeading(), CshootPose2.getHeading())
                .build();

        CpathToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(CshootPose2, CintakePose2))
                .setLinearHeadingInterpolation(CshootPose2.getHeading(), CintakePose2.getHeading())
                .build();

        CpathToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(CintakePose2, CshootPose3))
                .setLinearHeadingInterpolation(CintakePose2.getHeading(), CshootPose3.getHeading())
                .build();

        CpathToFinal = follower.pathBuilder()
                .addPath(new BezierLine(CshootPose3, CfinalPose))
                .setLinearHeadingInterpolation(CshootPose3.getHeading(), CfinalPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    protected void opLoop() {
        getID();
        follower.update();
        switch (tagID){
            case 21:
                buildPaths21();
                switch (pathState) {
                    case 0:
                        follower.followPath(pathToShoot1, true);
                        setPathState(1);
                        break;
                    case 1:
                        if (!follower.isBusy()) {
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(2);
                        }
                        break;
                    case 2:
                        if (pathTimer.seconds() > 0.5) {
                            transfer1Motor.setPower(1);
                            transfer2Motor.setPower(1);
                            pathTimer.reset();
                        }
                        else {
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            follower.followPath(pathToIntake1, true);
                            intakeMotor.setPower(1);
                            setPathState(3);
                        }
                        break;
                    case 3:
                        if (!follower.isBusy()) {
                            intakeMotor.setPower(0);
                            follower.followPath(pathToShoot2, true);
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(4);
                        }
                        break;
                    case 4:
                        if (!follower.isBusy()) {
                            if (pathTimer.seconds() > 0.5) {
                                transfer1Motor.setPower(1);
                                transfer2Motor.setPower(1);
                                pathTimer.reset();
                            }
                        } else {
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            follower.followPath(pathToIntake2, true);
                            intakeMotor.setPower(1);
                            setPathState(5);
                        }
                        break;
                    case 5:
                        if (!follower.isBusy()) {
                            intakeMotor.setPower(0);
                            follower.followPath(pathToShoot3, true);
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(6);
                        }
                        break;
                    case 6:
                        if (!follower.isBusy()) {
                            if (shotCount < 3) {
                                if (pathTimer.seconds() > 0.5) {
                                    transfer1Motor.setPower(1);
                                    transfer2Motor.setPower(1);
                                    pathTimer.reset();
                                }
                            } else {
                                transfer1Motor.setPower(0);
                                transfer2Motor.setPower(0);
                                flywheelMotor.setPower(0);
                                follower.followPath(pathToFinal, true);
                                setPathState(7);
                            }
                        }
                        break;


                    case 7:
                        if (!follower.isBusy()) {
                            flywheelMotor.setPower(0);
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            intakeMotor.setPower(0);
                            pathState = -1;
                        }
                        break;
                }
                break;
            case 22:
                buildPaths22();
                switch (pathState) {
                    case 0:
                        follower.followPath(BpathToShoot1, true);
                        setPathState(1);
                        break;
                    case 1:
                        if (!follower.isBusy()) {
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(2);
                        }
                        break;
                    case 2:
                        if (pathTimer.seconds() > 0.5) {
                            transfer1Motor.setPower(1);
                            transfer2Motor.setPower(1);
                            pathTimer.reset();
                        }
                        else {
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            follower.followPath(BpathToIntake1, true);
                            intakeMotor.setPower(1);
                            setPathState(3);
                        }
                        break;
                    case 3:
                        if (!follower.isBusy()) {
                            intakeMotor.setPower(0);
                            follower.followPath(BpathToShoot2, true);
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(4);
                        }
                        break;
                    case 4:
                        if (!follower.isBusy()) {
                            if (pathTimer.seconds() > 0.5) {
                                transfer1Motor.setPower(1);
                                transfer2Motor.setPower(1);
                                pathTimer.reset();
                            }
                        } else {
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            follower.followPath(BpathToIntake2, true);
                            intakeMotor.setPower(1);
                            setPathState(5);
                        }
                        break;
                    case 5:
                        if (!follower.isBusy()) {
                            intakeMotor.setPower(0);
                            follower.followPath(BpathToShoot3, true);
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(6);
                        }
                        break;
                    case 6:
                        if (!follower.isBusy()) {
                            if (shotCount < 3) {
                                if (pathTimer.seconds() > 0.5) {
                                    transfer1Motor.setPower(1);
                                    transfer2Motor.setPower(1);
                                    pathTimer.reset();
                                }
                            } else {
                                transfer1Motor.setPower(0);
                                transfer2Motor.setPower(0);
                                flywheelMotor.setPower(0);
                                follower.followPath(BpathToFinal, true);
                                setPathState(7);
                            }
                        }
                        break;


                    case 7:
                        if (!follower.isBusy()) {
                            flywheelMotor.setPower(0);
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            intakeMotor.setPower(0);
                            pathState = -1;
                        }
                        break;
                }
                break;
            case 23:
                buildPaths23();
                switch (pathState) {
                    case 0:
                        follower.followPath(CpathToShoot1, true);
                        setPathState(1);
                        break;
                    case 1:
                        if (!follower.isBusy()) {
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(2);
                        }
                        break;
                    case 2:
                        if (pathTimer.seconds() > 0.5) {
                            transfer1Motor.setPower(1);
                            transfer2Motor.setPower(1);
                            pathTimer.reset();
                        }
                        else {
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            follower.followPath(CpathToIntake1, true);
                            intakeMotor.setPower(1);
                            setPathState(3);
                        }
                        break;
                    case 3:
                        if (!follower.isBusy()) {
                            intakeMotor.setPower(0);
                            follower.followPath(CpathToShoot2, true);
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(4);
                        }
                        break;
                    case 4:
                        if (!follower.isBusy()) {
                            if (pathTimer.seconds() > 0.5) {
                                transfer1Motor.setPower(1);
                                transfer2Motor.setPower(1);
                                pathTimer.reset();
                            }
                        } else {
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            follower.followPath(CpathToIntake2, true);
                            intakeMotor.setPower(1);
                            setPathState(5);
                        }
                        break;
                    case 5:
                        if (!follower.isBusy()) {
                            intakeMotor.setPower(0);
                            follower.followPath(CpathToShoot3, true);
                            flywheelMotor.setPower(flywheelPower);
                            setPathState(6);
                        }
                        break;
                    case 6:
                        if (!follower.isBusy()) {
                            if (shotCount < 3) {
                                if (pathTimer.seconds() > 0.5) {
                                    transfer1Motor.setPower(1);
                                    transfer2Motor.setPower(1);
                                    pathTimer.reset();
                                }
                            } else {
                                transfer1Motor.setPower(0);
                                transfer2Motor.setPower(0);
                                flywheelMotor.setPower(0);
                                follower.followPath(CpathToFinal, true);
                                setPathState(7);
                            }
                        }
                        break;


                    case 7:
                        if (!follower.isBusy()) {
                            flywheelMotor.setPower(0);
                            transfer1Motor.setPower(0);
                            transfer2Motor.setPower(0);
                            intakeMotor.setPower(0);
                            pathState = -1;
                        }
                        break;
                }
                break;
        }
    }
    public int getID(){
        LLResult id = limelight.getLatestResult();

        if (id != null && id.isValid()){
            for (LLResultTypes.FiducialResult fr : id.getFiducialResults()){
                if (fr != null){
                    return fr.getFiducialId();
                }
            }
        }

        return -1;
    }
    @Override
    protected void opTeardown(){
    }
}




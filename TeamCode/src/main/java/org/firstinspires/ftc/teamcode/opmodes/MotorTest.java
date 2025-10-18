package org.firstinspires.ftc.teamcode.opmodes;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.List;
@Config
@TeleOp(name = "MotorTest")
public class MotorTest extends OpMode {
    public static double motorpower = 0.5;
    public static double motorpower2 = -0.5;

    private DcMotorEx motorLeft, motorRight;

    public static String motorLeftName = "outtake1Motor";
    public static String motorRightName = "outtake2Motor";

    @Override
    public void init() {
        motorLeft = hardwareMap.get(DcMotorEx.class, motorLeftName);
        motorRight = hardwareMap.get(DcMotorEx.class, motorRightName);

    }


    @Override
    public void loop() {
        motorRight.setPower(motorpower);
        motorLeft.setPower(motorpower2);
    }

}

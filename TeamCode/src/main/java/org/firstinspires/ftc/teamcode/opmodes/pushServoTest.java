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
@TeleOp(name = "pushServoTest")
public class pushServoTest extends OpMode {
    public static double holdPosL = 0.5;
    public static double holdPosR = 0.5;

    private Servo rightpushServo, leftpushServo;

    public static String servoLeftName = "leftpushServo";
    public static String servoRightName = "rightpushServo";

    @Override
    public void init() {
        leftpushServo = hardwareMap.get(Servo.class, servoLeftName);
        rightpushServo = hardwareMap.get(Servo.class, servoRightName);

    }


    @Override
    public void loop() {
        leftpushServo.setPosition(holdPosL);
        rightpushServo.setPosition(holdPosR);
    }

}
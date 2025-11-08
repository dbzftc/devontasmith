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
@TeleOp(name = "ServoTest")
public class holdTest extends OpMode {
    public static double holdPos = 0.5;
    public static double shootPos = 0.5;

    private Servo holdServo;
    private Servo shoot1Servo;
    private Servo shoot2Servo;



    @Override
    public void init() {
        holdServo = hardwareMap.get(Servo.class, "holdServo");
        shoot1Servo = hardwareMap.get(Servo.class, "shoot1Servo");
        shoot2Servo = hardwareMap.get(Servo.class, "shoot2Servo");

    }


    @Override
    public void loop() {
        holdServo.setPosition(holdPos);
        shoot1Servo.setPosition(shootPos);
        shoot2Servo.setPosition(shootPos);



    }
}

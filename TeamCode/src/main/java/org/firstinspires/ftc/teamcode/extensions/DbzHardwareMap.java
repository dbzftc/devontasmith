package org.firstinspires.ftc.teamcode.extensions;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DbzHardwareMap {

    public DcMotorEx frontRight, backRight, backLeft, frontLeft;
    public DcMotorEx hoodMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx transfer1Motor;
    public DcMotorEx transfer2Motor;
    public DcMotorEx flywheelMotor;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public Limelight3A limelight;
    private Follower follower;

    public enum Motor {
        frontright("frontRight"),
        backright("backRight"),
        backleft("backLeft"),
        frontleft("frontLeft"),
        hood("hoodMotor"),
        intake("intakeMotor"),
        transfer1("transfer1Motor"),
        transfer2("transfer2Motor"),
        flywheel("flywheelMotor");

        private final String name;
        Motor(String name) { this.name = name; }
        public String getName() { return name; }
    }

    public DbzHardwareMap(HardwareMap hwMap) {
        frontRight = hwMap.get(DcMotorEx.class, Motor.frontright.getName());
        backRight = hwMap.get(DcMotorEx.class, Motor.backright.getName());
        backLeft = hwMap.get(DcMotorEx.class, Motor.backleft.getName());
        frontLeft = hwMap.get(DcMotorEx.class, Motor.frontleft.getName());

        hoodMotor = hwMap.get(DcMotorEx.class, Motor.hood.getName());
        intakeMotor = hwMap.get(DcMotorEx.class, Motor.intake.getName());
        transfer1Motor = hwMap.get(DcMotorEx.class, Motor.transfer1.getName());
        transfer2Motor = hwMap.get(DcMotorEx.class, Motor.transfer2.getName());
        flywheelMotor = hwMap.get(DcMotorEx.class, Motor.flywheel.getName());

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        limelight = hwMap.get(Limelight3A.class, "limelight");
    }
}
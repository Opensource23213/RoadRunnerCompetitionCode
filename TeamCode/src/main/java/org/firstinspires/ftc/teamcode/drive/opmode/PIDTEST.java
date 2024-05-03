package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Encoder;


@Config
@TeleOp(name="PIDTEST", group="ABC Opmode")
//@Disabled
public class PIDTEST extends OpMode {
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int target = 0;

    public final double ticks_in_degree = 22.76;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx Arm1 = null;

    private DcMotorEx Arm2 = null;

    private DcMotor ArmPos = null;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm1 = hardwareMap.get(DcMotorEx.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotorEx.class, "Arm2");
        ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        Arm1.setDirection(DcMotor.Direction.REVERSE);
        Arm2.setDirection(DcMotor.Direction.FORWARD);
    }
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPose = ArmPos.getCurrentPosition();
        double pid = controller.calculate(armPose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        Arm1.setPower(power);
        Arm2.setPower(power);

        telemetry.addData("pose", armPose);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.update();
    }
}




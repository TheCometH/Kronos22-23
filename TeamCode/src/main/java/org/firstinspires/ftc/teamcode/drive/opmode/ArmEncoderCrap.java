package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ArmEncoderCrap extends LinearOpMode {

    //DcMotorEx arm;
    DcMotorEx expansion;
    DcMotorEx rotation;
    DcMotorEx traverse;
    Servo tilt;
    Servo clamp;
    
    @Override
    public void runOpMode() throws InterruptedException {
        expansion = hardwareMap.get(DcMotorEx.class, "expansion");
        rotation = hardwareMap.get(DcMotorEx.class, "rotation");
        traverse = hardwareMap.get(DcMotorEx.class, "traverse");
        tilt = hardwareMap.get(Servo.class, "tilt");
        clamp = hardwareMap.get(Servo.class, "clamp");

        expansion.setDirection(DcMotorSimple.Direction.FORWARD);
        rotation.setDirection(DcMotorSimple.Direction.FORWARD);
        traverse.setDirection(DcMotorSimple.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        clamp.setDirection(Servo.Direction.FORWARD);


        expansion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        traverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // trajectory to tall pole (theoretically)
        
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(12, 46.6666666666))
                .splineTo(new Vector2d(12, 72), 0)
                .addDisplacementMarker( () -> {
                    // estimations on 135 deg to roate it to the right height 
                    Rotate(0.5, 540);
                    // change the distance to however much the expansion needs
                    Expansion(0.1, 2880);
                    Stop();
             })
                .splineTo(new Vector2d(48, 72), Math.toRadians(90))
                .build();
        waitForStart();
        if(isStopRequested()) return;
        drive.followTrajectory(traj1);
    }

    public void Stop() {
        expansion.setPower(0);
        rotation.setPower(0);
        traverse.setPower(0);
    }

    public void Rotate(double power, int distance) {
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODERS);

        rotation.setTargetPosition(distance);

        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotation.setPower(power);

        while (rotation.isBusy()) {

        }

        Stop();
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Expansion(double power, int distance) {
        expansion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        expansion.setTargetPosition(distance);

        expansion.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        expansion.setPower(power);

        while (expansion.isBusy()) {

        }

        Stop();
        expansion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Traverse(double power, int distance) {
        traverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        traverse.setTargetPosition(distance);

        traverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        traverse.setPower(power);
        while (traverse.isBusy()) {

        }

        Stop();
        traverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Tilt(double distance) {
        tilt.setPosition(distance);
    }

    public void CloseClaw(double distance) {
        clamp.setPosition(distance);
    }

    public void OpenClaw() {
        clamp.setPosition(0);
    }
}

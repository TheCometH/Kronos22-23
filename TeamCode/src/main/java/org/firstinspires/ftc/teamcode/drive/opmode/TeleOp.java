package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TeleOp extends LinearOpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


    @Override
    public void runOpMode() {

        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.left_stick_y > 0.4){
                drive.move(1);
            }
            if (gamepad1.left_stick_y < -0.4){
                drive.move(-1);
            }
            if (gamepad1.left_stick_x > 0.4){
                drive.sides(1);
            }
            if (gamepad1.left_stick_x < -0.4){
                drive.sides(-1);
            }
            if(gamepad1.right_bumper){
                drive.turning(1);
            }
            if(gamepad1.left_bumper){
                drive.turning(-1);
            }

            if(gamepad2.a){
                drive.openClaw();

            }
            if(gamepad2.b){
                drive.closeClaw(0.2);
                // unknown distance

            }
            if(gamepad2.x){

                drive.traversing(0.1);
            }
            if(gamepad2.y){

                drive.traversing(-0.1);
            }
            if(gamepad2.right_stick_y > 0.4){
                drive.expand(-0.1);
            }
            if(gamepad2.right_stick_y < -0.4){
                drive.expand(0.1);
            }
            if(gamepad2.left_stick_y < 0.4){
                drive.rotate(1);
            }
            if(gamepad2.left_stick_y < -0.4){
                drive.rotate(-1);
            }
            if (gamepad2.left_bumper) {
                drive.tiltNow(0.2);
            }
            if (gamepad2.right_bumper){
                drive.tiltNow(-0.2);

            }


        }
    }
}

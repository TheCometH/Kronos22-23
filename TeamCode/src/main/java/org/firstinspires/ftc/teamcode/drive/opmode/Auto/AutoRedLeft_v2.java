package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.cameradetection.CameraDetection;

public class AutoRedLeft_v2 extends LinearOpMode {
    @Override
    public void runOpMode() {

        //Creates a CameraDetection object to detect the id and return a value for parking
        CameraDetection camera = new CameraDetection();
        camera.init();
        camera.detect();
        camera.update();

        //Initialize park to make it equal
        int park = camera.check();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(72, -36, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(48)
                .splineTo(new Vector2d(21, -36), Math.toRadians(90))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(21,-54), Math.toRadians(180))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(21,-36), Math.toRadians(180))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(21,-54), Math.toRadians(180))
                .build();

        Trajectory parkTraj;

        if (park == 1) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .forward(24)
                    .build();
        }
        else if (park == 2) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .build();
        }
        else if (park == 3) {
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .back(24)
                    .build();
        }
        else {
            telemetry.addLine("well frick");
            telemetry.update();
            parkTraj = drive.trajectoryBuilder(traj3.end())
                    .build();
        }

        //Goes to -21, -36, turning 90 degrees clockwise
        drive.followTrajectory(traj1);

        //Code to drop the cone on the top junction
        //drive.rotate(0.1, 1);
        //drive.traversing(0.1, 1);
        //drive.expand(0.1, 1);
        //drive.tiltNow(90);
        //drive.openClaw();

        //Brings the
        //drive.tiltNow();
        //drive.traversing(-0.1, 1);
        //drive.expand(-0.05, 1);

        drive.followTrajectory(traj2); //rotate arm -90 degrees during
        //drive.expand(0.1, 1);
        //drive.rotate(0.1, -0.25);
        //drive.closeClaw();
        //drive.rotate(0.1, 0.25);
        //drive.expand(0.1, 1);
        //drive.tiltNow();
        //drop cone on short junction
        //rotate arm -135 degrees
        //grab cone
        drive.followTrajectory(traj3);
        //drive.traversing(0.1, 1);
        //drive.expand(0.1, 1);
        //drive.tiltNow(90);
        //drive.openClaw();
        //drive.tiltNow();
        //drive.traversing(0.1, -1);
        //drive.expand(0.1, -1);
        //drop on tall junction
        //park``
        drive.followTrajectory(traj4);
        //drive.expand(0.1, 1);
        //drive.rotate(0.1, -0.25);
        //drive.closeClaw();
        //drive.rotate(0.1, 0.25);
        //drive.expand(0.1, 1);
        //drive.tiltNow();

        drive.followTrajectory(parkTraj);
    }
}

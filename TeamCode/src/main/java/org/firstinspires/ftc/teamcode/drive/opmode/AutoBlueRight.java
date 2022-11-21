public class AutoBlueRight extends LinearOpMode{
  @Override
  public void runOpMode() {
    int camera = 0;
          
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
  
    Pose2d startPose = new Pose2d(-72, -36, Math.toRadians(0));
    
    Trajectory traj1 = drive.trajectoryBuilder(new Pose2D())
      .splineTo(new Vector2d(-60,-36), Math.toRadians(0))
      .build();
  
    Trajectory traj2 = drive.trajectoryBuilder(new traj1.end())
      .splineTo(new Vector2d(-60,-60), Math.toRadians(0))
      .splineTo(new Vector2d(-33,-60), Math.toRadians(0))
      .build();
  
    Trajectory traj3 = drive.trajectoryBuilder(new traj2.end())
      .splineTo(new Vector2d(-12,-36), Math.toRadians(90))
      .build();
  
    Trajectory park;
      
      if (camera == 1) {
          park = drive.trajectoryBuilder(new traj3.end())
            .forward(24)
            .build();
      }
      else if (camera == 2) {
          park = drive.trajectoryBuilder(new traj3.end())
            .build();
      }
      else if (camera == 3) {
          park = drive.trajectoryBuilder(new traj3.end())
            .back(24)
            .build();
      }
      else {
        telemetry.add("well frick");
        telemetry.update();
      }
      
    drive.followTrajectory(traj1);
    //scan with camera
    drive.followTrajectory(traj2); //roate arm -90 degrees during
    //drop cone on short junction
    //rotate arm -135 degrees
    //grab cone
    drive.followTrajectory(traj3);
    //drop on tall junction
    //park
    drive.followTrajectory(park);
    }
}

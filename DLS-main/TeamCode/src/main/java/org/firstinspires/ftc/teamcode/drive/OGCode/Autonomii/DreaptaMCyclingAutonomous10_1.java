package org.firstinspires.ftc.teamcode.drive.OGCode.Autonomii;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.timeOutBaby;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1_LimitTesting;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1_LimitTesting;
import org.firstinspires.ftc.teamcode.drive.OGCode.BiggerController;
import org.firstinspires.ftc.teamcode.drive.OGCode.CloseClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController;
import org.firstinspires.ftc.teamcode.drive.OGCode.PipeLineDetector;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.SigurantaLiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;


@Config
@Autonomous(group = "drive")

public class DreaptaMCyclingAutonomous10_1 extends LinearOpMode {
    enum STROBOT
    {
        START,
        FIRST_CYCLE,
        SECOND_CYCLE,
        THIRD_CYCLE,
        FOURTH_CYCLE,
        FIFTH_CYCLE,
        SIXTH_CYCLE,
        RETRACT,
        PARK,
        STOP_JOC,
        PRELOAD,
        GO_TO_NEXT_POSITION,
        GET_DOWN
    }
    public static double x_CYCLING_POSITION = 38.3, y_CYCLING_POSITION = -23, Angle_CYCLING_POSITION = 13.5;
    public static double x_INTER = 35.5, y_INTER = -35 , Angle_PARK_INTER = 270;
    public static double x_PARK1 = 10, y_PARK1 = -35, Angle_PARK1 = 270;
    public static double x_PARK2 = 36.5, y_PARK2 = -35, Angle_PARK2 = 270;
    public static double x_PARK3 = 57, y_PARK3 = -35, Angle_PARK3 = 270;
    ElapsedTime asteapta = new ElapsedTime(), timerRetract = new ElapsedTime(), timerLift =new ElapsedTime();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        timeOutBaby = 0.5;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Servo4BarController servo4BarController = new Servo4BarController();
        MotorColectareController motorColectareController = new MotorColectareController();
        CloseClawController closeClawController = new CloseClawController();
        TurnClawController turnClawController = new TurnClawController();
        RobotController robotController = new RobotController();
        BiggerController biggerController = new BiggerController();
        LiftController liftController = new LiftController();
        AutoController10_1_LimitTesting autoController101 = new AutoController10_1_LimitTesting();
        SigurantaLiftController sigurantaLiftController = new SigurantaLiftController();
        Angle4BarController angle4BarController = new Angle4BarController();
        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
        robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;



        autoController101.Cone_Stack_Level  = 5;
        autoController101.AutoLiftStatus = LiftController.LiftStatus.MID;
        autoController101.LimitLift = 0.75;
        autoController101.timerMiscareBaby = 0.9;

        angle4BarController.update(robot);
        closeClawController.update(robot);
        turnClawController.update(robot);
        servo4BarController.update(robot);
        sigurantaLiftController.update(robot);
        motorColectareController.update(robot,0, 1, currentVoltage);
        liftController.update(robot,0,sigurantaLiftController,currentVoltage);
        robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
        sigurantaLiftController.update(robot);
        int nr=0;
        Pose2d startPose = new Pose2d(35, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectorySequence PLACE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x_CYCLING_POSITION,y_CYCLING_POSITION,Math.toRadians(Angle_CYCLING_POSITION)))
                .build();
        TrajectorySequence PARK1 = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK1,y_PARK1,Math.toRadians(Angle_PARK1)))
                .build();
        TrajectorySequence PARK2 = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK2,y_PARK2,Math.toRadians(Angle_PARK2)))
                .build();
        TrajectorySequence PARK3 = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineToLinearHeading(new Pose2d(x_INTER,y_INTER,Math.toRadians(Angle_PARK_INTER)))
                .lineToLinearHeading(new Pose2d(x_PARK3,y_PARK3,Math.toRadians(Angle_PARK3)))
                .build();
        TrajectorySequence GO_TO_NEXT_POSITION = drive.trajectorySequenceBuilder(PLACE_PRELOAD.end())
                .lineTo(new  Vector2d(35, -21))
                .splineToSplineHeading(new Pose2d(10,-10,Math.toRadians(180)),Math.toRadians(180))
                .lineTo(new Vector2d(-20,-10))
                .splineToSplineHeading(new Pose2d(-35,-23,Math.toRadians(160)),Math.toRadians(270))
                .build();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addData("Tag of interest is in sight!\n\nLocation data:", tagOfInterest.id);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addData("\nBut we HAVE seen the tag before; last seen at:" , tagOfInterest.id);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addData("\nBut we HAVE seen the tag before; last seen at:" , tagOfInterest.id);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addData("Tag snapshot:\n" , tagOfInterest.id);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (!isStarted()&&!isStopRequested())
        {
            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            int ColectarePosition = robot.motorColectareStanga.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista
            switch (status)
            {
                case START:
                {
                    drive.followTrajectorySequenceAsync(PLACE_PRELOAD);
                    status = STROBOT.PRELOAD;
                    break;
                }
                case PRELOAD:
                {
                    if (!drive.isBusy())
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.MID;
                        servo4BarController.Collect_Position = servo4BarController.Fifth_Cone_Position_MID;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                        timerLift.reset();
                        status = STROBOT.FIRST_CYCLE;
                    }
                    break;
                }
                case FIRST_CYCLE:
                {
                    if (timerLift.seconds()>0.75)
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                        autoController101.CurrentStatus = AutoController10_1_LimitTesting.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.SECOND_CYCLE;
                    }
                    break;
                }
                case SECOND_CYCLE:
                {
                    if (autoController101.CurrentStatus == AutoController10_1_LimitTesting.autoControllerStatus.NOTHING)
                    {
                        autoController101.CurrentStatus = AutoController10_1_LimitTesting.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.THIRD_CYCLE;
                    }
                    break;
                }
                case THIRD_CYCLE:
                {
                    if (autoController101.CurrentStatus == AutoController10_1_LimitTesting.autoControllerStatus.NOTHING)
                    {
                        autoController101.CurrentStatus = AutoController10_1_LimitTesting.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.FOURTH_CYCLE;
                    }
                    break;
                }
                case FOURTH_CYCLE:
                {
                    if (autoController101.CurrentStatus == AutoController10_1_LimitTesting.autoControllerStatus.NOTHING)
                    {
                        autoController101.CurrentStatus = AutoController10_1_LimitTesting.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.FIFTH_CYCLE;
                    }
                    break;
                }
                case FIFTH_CYCLE:
                {
                    if (autoController101.CurrentStatus == AutoController10_1_LimitTesting.autoControllerStatus.NOTHING)
                    {
                        autoController101.CurrentStatus = AutoController10_1_LimitTesting.autoControllerStatus.STACK_LEVEL;
                        status = STROBOT.RETRACT;
                    }
                    break;
                }
                case RETRACT:
                {
                    if (autoController101.CurrentStatus == AutoController10_1_LimitTesting.autoControllerStatus.NOTHING)
                    {
                        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        timerRetract.reset();
                        status = STROBOT.GO_TO_NEXT_POSITION;
                    }
                    break;
                }
                case GO_TO_NEXT_POSITION:
                {
                     drive.followTrajectorySequenceAsync(GO_TO_NEXT_POSITION);
                     status = STROBOT.SIXTH_CYCLE;
                     break;
                }
                case SIXTH_CYCLE:
                {
                     if (!drive.isBusy())
                     {
                         autoController101.CurrentStatus = AutoController10_1_LimitTesting.autoControllerStatus.STACK_LEVEL;
                         status = STROBOT.STOP_JOC;
                     }
                     break;
                }
                case PARK:
                {
                    if (timerRetract.seconds()>0.3)
                    {
                        if (tagOfInterest == null)
                        {
                            drive.followTrajectorySequenceAsync(PARK2);
                        }
                        else
                        {
                            if (tagOfInterest.id  == LEFT)
                            {
                                drive.followTrajectorySequenceAsync(PARK1);
                            }
                            else
                            if (tagOfInterest.id == MIDDLE)
                            {
                                drive.followTrajectorySequenceAsync(PARK2);
                            }
                            else
                            {
                                drive.followTrajectorySequenceAsync(PARK3);
                            }
                        }
                        status = STROBOT.STOP_JOC;
                    }
                    break;
                }
            }
            biggerController.update(robotController,closeClawController,motorColectareController);
            robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
            closeClawController.update(robot);
            angle4BarController.update(robot);
            turnClawController.update(robot);
            servo4BarController.update(robot);
            sigurantaLiftController.update(robot);
            motorColectareController.update(robot,ColectarePosition, 1, currentVoltage);
            liftController.update(robot,LiftPosition,sigurantaLiftController,currentVoltage);
            autoController101.update(sigurantaLiftController,robot,angle4BarController, turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);

            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            // telemetry.addData("caz:", Case);
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}
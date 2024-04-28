package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.COLLECT_DRIVE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.DRIVE_POSITION;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.INTERMEDIARY;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.PLACE_CONE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Servo4BarController {
    public enum ServoStatus
    {
        INITIALIZE,
        COLLECT_DRIVE,
        INTERMEDIARY,
        PLACE_CONE,
        DRIVE_POSITION,
        STACK_POSITION,
        FALLEN_CONES,
        LOW_POSITION,
    }
    public static ServoStatus CurrentStatus = INITIALIZE,PreviousStatus = INITIALIZE,WhereFromIntermediary = COLLECT_DRIVE;
    ElapsedTime time = new ElapsedTime();
    public static double Ground_Position=0.925, Second_Cone_Position=0.885, Third_Cone_Position=0.845, Fourth_Cone_Position = 0.8, Fifth_Cone_Position = 0.775,
             Fifth_Cone_Position_South_Left = 0.75 , Fifth_Cone_Position_MID = 0.75,
             Fifth_Cone_Position_BRATJOSHIGH = 0.75, Fifth_Cone_Position_BRATJOSMID_2 = 0.75, groundJunctionPosition  = 0.91;
    public static double Collect_Position = 0.85 , Place_Cone_Position = 0.26, Intermediary_Position =0.5 , Drive_Position = 0.5;
    public static double Collect_Drive = 0.932 , Low_Position = 0.6;
    public static double Fallen_Cones = 0.955;
    public static double Fifth_Cone_Position_BRATJOSHIGH_2 = 0.75, Fourth_Cone_Position_BRATJOSHIGH_2 = 0.795 , Third_Cone_Position_BRATJOSHIGH_2 = 0.84, Second_Cone_Position_BRATJOSHIGH_2 = 0.88, Ground_Position_BRATJOSHIGH_2 = 0.91;

    public static double Fifth_Cone_Position_BRATJOSMID = 0.75;

    public static double Ground_Position_South=0.915, Second_Cone_Position_South=0.885, Third_Cone_Position_South=0.845, Fourth_Cone_Position_South = 0.8, Fifth_Cone_Position_South = 0.755;

    public static double CyclingSouthPositions[] = {0,0.925,0.885,0.845,0.8,0.755};
    public static double StackPositions[] = {0,0.94,0.9,0.86,0.82,0.775};
    public static double GroundPositions[] = {0,0,0.76,0.70,0.64,0.56};
    int salut =0;
    public void update(RobotMap Robot)
    {
        if (PreviousStatus != CurrentStatus || CurrentStatus==INTERMEDIARY || CurrentStatus==INITIALIZE)
        {
            switch(CurrentStatus)
            {
                case INITIALIZE:
                {
                    Robot.left4Bar.setPosition(Drive_Position);
                    Robot.right4Bar.setPosition(Drive_Position);
                    break;
                }
                case INTERMEDIARY:
                {
                    salut = 2;
                    if (WhereFromIntermediary == COLLECT_DRIVE)
                    {
                            Robot.right4Bar.setPosition(Place_Cone_Position);
                            Robot.left4Bar.setPosition(Place_Cone_Position);
                            CurrentStatus = PLACE_CONE;
                    }
                    else {
                            Robot.right4Bar.setPosition(Collect_Position);
                            Robot.left4Bar.setPosition(Collect_Position);
                            CurrentStatus = COLLECT_DRIVE;
                    }
                    break;
                }
                case COLLECT_DRIVE:
                {
                    if (PreviousStatus == PLACE_CONE)
                    {
                        time.reset();
                        Robot.left4Bar.setPosition(Intermediary_Position);
                        Robot.right4Bar.setPosition(Intermediary_Position);
                        WhereFromIntermediary = PLACE_CONE;
                        CurrentStatus = INTERMEDIARY;
                    }
                    else if (PreviousStatus == INITIALIZE)
                    {
                        time.reset();
                        Robot.left4Bar.setPosition(Collect_Position);
                        Robot.right4Bar.setPosition(Collect_Position);
                    }
                    break;
                }
                case PLACE_CONE:
                {
                    time.reset();
                    if (PreviousStatus == COLLECT_DRIVE)
                    {
                        salut=1;
                        Robot.left4Bar.setPosition(Intermediary_Position);
                        Robot.right4Bar.setPosition(Intermediary_Position);
                        WhereFromIntermediary = COLLECT_DRIVE;
                        CurrentStatus = INTERMEDIARY;
                    }
                    break;
                }
                case DRIVE_POSITION:
                {
                    Robot.left4Bar.setPosition(Drive_Position);
                    Robot.right4Bar.setPosition(Drive_Position);
                    CurrentStatus = PLACE_CONE;
                    break;
                }
                case LOW_POSITION:
                {
                    Robot.left4Bar.setPosition(Low_Position);
                    Robot.right4Bar.setPosition(Low_Position);
                    CurrentStatus = PLACE_CONE;
                    break;
                }
                case FALLEN_CONES:
                {
                    Robot.left4Bar.setPosition(Fallen_Cones);
                    Robot.right4Bar.setPosition(Fallen_Cones);
                    CurrentStatus = COLLECT_DRIVE;
                    break;
                }
                case STACK_POSITION:
                {
                    CurrentStatus = COLLECT_DRIVE;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }

}

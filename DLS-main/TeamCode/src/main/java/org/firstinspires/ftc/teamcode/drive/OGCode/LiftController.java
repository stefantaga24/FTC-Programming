package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.LiftController.LiftStatus.BASE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.LiftController.LiftStatus.PLACE_SIGURANTA;
import static org.firstinspires.ftc.teamcode.drive.OGCode.LiftController.LiftStatus.PLACE_SIGURANTA_INTF;
import static org.firstinspires.ftc.teamcode.drive.OGCode.LiftController.LiftStatus.START;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LiftController {
    public enum LiftStatus
    {
        START,
        HIGH_SOUTH,
        HIGH_SOUTH_LEFT,
        HIGH,
        HIGH_DRIVE,
        LOW,
        MID,
        PLACE_SIGURANTA,
        BASE_BAZAVAN,
        HIGH_SOUTH_CYCLING,
        INTF,
        PLACE_SIGURANTA_INTF,
        BASE,
        BASE_CU_SIGURANTA,
        BASE_INTF,
        MIDAUTO,
        PLM,
    }
    public double CurrentSpeed=0;
    public double Kp = 0.008;
    public double Ki = 0.0055;
    public double Kd = 0;
    public double Kg = 0;
    public double maxSpeed = 1;
    public static LiftStatus CurrentStatus = START, PreviousStatus = START;
    SimplePIDController LiftColectarePID = null;
    ElapsedTime timeSiguranta = new ElapsedTime(), timeSiguranta2 = new ElapsedTime();
    /// pe DreaptaLift am encoder
    int basePosition = 0;
    public static int lowPosition = 207;
    public static int midPosition = 420, midAuto = 410;
    public static int highPosition = 670, highPositionSouth = 680, highPosition_DRIVE = 680 , highPositionSouth_Cycling = 705,highPositionSouth_LEFT = 700;
    public int CurrentPosition = 0;
    public LiftController()
    {
        LiftColectarePID = new SimplePIDController(Kp,Ki,Kd);
        LiftColectarePID.targetValue=basePosition;
        LiftColectarePID.maxOutput = maxSpeed;
    }
    public void update(RobotMap Robotel, int LiftPosition, SigurantaLiftController sigurantaLiftController, double CurrentVoltage)
    {
        CurrentPosition = LiftPosition;
        double powerLift = LiftColectarePID.update(LiftPosition) + Kg;
        powerLift = Math.max(-1,Math.min(powerLift* 14 / CurrentVoltage,1));
        CurrentSpeed=powerLift;
        Robotel.stangaLift.setPower(powerLift);
        Robotel.dreaptaLift.setPower(powerLift);
        if (CurrentStatus != PreviousStatus || CurrentStatus == PLACE_SIGURANTA || CurrentStatus == PLACE_SIGURANTA_INTF)
        {
            switch (CurrentStatus)
            {
                case BASE:
                {
                    LiftColectarePID.targetValue = basePosition;
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
                    break;
                }
                case PLM:
                {
                    LiftColectarePID.targetValue = 500;
                    break;
                }
                case BASE_INTF:
                {
                    LiftColectarePID.targetValue = basePosition;
                    timeSiguranta2.reset();
                    CurrentStatus = PLACE_SIGURANTA_INTF;
                    break;
                }

                case BASE_BAZAVAN:
                {
                    LiftColectarePID.targetValue = basePosition;
                    timeSiguranta.reset();
                    CurrentStatus = PLACE_SIGURANTA;
                    break;
                }
                case PLACE_SIGURANTA:
                {
                    if (timeSiguranta.seconds()>0.25)
                    {
                        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
                        CurrentStatus = BASE;
                    }
                    break;
                }

                case BASE_CU_SIGURANTA: {
                    LiftColectarePID.targetValue = basePosition;
                    break;
                }

                case PLACE_SIGURANTA_INTF:
                {
                    if(timeSiguranta2.seconds() > 0.15)
                    {
                        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
                        CurrentStatus = BASE;
                    }
                    break;
                }

                case HIGH:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = highPosition;
                    break;
                }
                case HIGH_DRIVE:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = highPosition_DRIVE;
                    break;
                }
                case HIGH_SOUTH:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = highPositionSouth;
                    break;
                }
                case HIGH_SOUTH_LEFT:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = highPositionSouth_LEFT;
                    break;
                }
                case HIGH_SOUTH_CYCLING:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = highPositionSouth_Cycling;
                    break;
                }
                case MID:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = midPosition;
                    break;
                }

                case  MIDAUTO: {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = midAuto;
                    break;
                }
                case LOW:
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    LiftColectarePID.targetValue = lowPosition;
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}

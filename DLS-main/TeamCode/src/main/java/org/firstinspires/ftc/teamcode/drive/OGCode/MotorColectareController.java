package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_1050;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_2050;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.INITIALIZED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class MotorColectareController {
    public enum MotorColectare
    {
        INITIALIZED,
        EXTENDED,
        RETRACTED,
        RETRACTED_0,
        EXTENDED_FAST,
        EXTENDED_5_1,
        EXTENDED_5_1_LEFT,
        EXTENDED_SOUTH_SIGUR,
        EXTENDED_10_1,
        EXTENDED_10_1_SOUTH,
        EXTENDED_COMMONHIGHINTERFERENCE,
        EXTENDED_SOUTH_LEFT,
        EXTENDED_SOUTH_DREAPTA_HAZ_2,
        EXTENDED_DRIVE,
        CLOSE_TO_EXTENDED_FIRST_CONE,
        EXTENDED_FIRST_CONE,
        EXTENDED_1050,
        HALF_WAY,
        THREE_WAY,
        EXTENDED_SOUTH,
        EXTENDED_2050,
        Colectare_4bar,
        Colectare_siguranta,
        Colectare_close_claw,
        Colectare_liftsanki,
        SA_VA_DAU_LA_MUIE,
        PulaMea,
        DRAW_MOTOR1,
        DRAW_MOTOR2,
        MID_CU_INTF_IN_PULA_MEA,
        DRAW_MOTOR_MID_1,
        DRAW_MOTOR_MID_2,
        BREAK_BEAM_INTERMEDIATE,
    }
    public static double Kp = 0.00325;
    public static double Ki = 0.0022;
    public static double Kd = 0;
    public static double maxSpeed = 1;
    public static MotorColectare CurrentStatus = INITIALIZED,  PreviousStatus = INITIALIZED;
    SimplePIDController MotorColectarePID = null;
    public static double vMax = 0, AccMax = 0, JerkMax =0 , EndPos = 2020 , CurrentPosition = 0;
    public static double extendedPosition = 497 , retractedPosition = -30, extendedDrive = 655, extendedDriveMax = 655;
    public static int extended10_1Autonomy_10= 325, extended10_1Autonomy_9 = 312,extended10_1Autonomy_8 = 305,
                      extended10_1Autonomy_7 = 298, extended10_1Autonomy_6 = 298, extended10_1Autonomy_5 = 312,
                      extended10_1Autonomy_4= 301 , extended10_1Autonomy_3 = 298,extended10_1Autonomy_2 = 291,
                      extended10_1Autonomy_1 = 291;
    public static double ExtendoCyclingSouthPositions[] = {520,520,535,555,590, 520,520,535,555,590};
    public static double Extendo5_1SouthPositions[] = {500,500,515,525,535};
    public static double Extendo5_1Haz2DreaptaSouthPositions[] = {475,475,485,505,545};
    public static double Extendo5_1SouthLeftPositions[] = {495,495,515,525,545};
    public static double ExtendoHaz[]= {0,310,310,312,319,340};
    public static double ExtendoHazLeft[] = {0,310,310,312,319,340};
    public static double MIDCUINTFINPULAMEA[] = {0, 110, 120, 130, 140, 150};
    public static double VLT[] = {0.55, 0.55, 0.55, 0.55, 0.55};
    public static double VLT2[] = {0.35, 0.35, 0.35, 0.35, 0.35};
    public static int NrConAuto = 5;
    public MotorColectareController()
    {
        MotorColectarePID = new SimplePIDController(Kp,Ki,Kd);
        MotorColectarePID.targetValue=retractedPosition;
        MotorColectarePID.maxOutput = maxSpeed;
    }
    public void update(RobotMap Robotel, int ColectarePosition, double PowerCap, double CurrentVoltage)
    {
        CurrentPosition = ColectarePosition;
        double powerColectare = MotorColectarePID.update(ColectarePosition);
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare* 14 / CurrentVoltage,PowerCap));
        Robotel.motorColectareStanga.setPower(powerColectare);
        Robotel.motorColectareDreapta.setPower(powerColectare);
        if (PreviousStatus != CurrentStatus || CurrentStatus == EXTENDED_1050 || CurrentStatus == EXTENDED || CurrentStatus == EXTENDED_2050)
        {
            switch (CurrentStatus)
            {
                case RETRACTED:
                {
                    MotorColectarePID.targetValue = retractedPosition;
                    MotorColectarePID.maxOutput = 1;
                    break;
                }
                case DRAW_MOTOR1:
                {
                    MotorColectarePID.targetValue = 700;
                    MotorColectarePID.maxOutput = VLT[NrConAuto];
                    break;
                }
                case DRAW_MOTOR2: {
                    MotorColectarePID.targetValue = 650;
                    MotorColectarePID.maxOutput = 0.45;
                    break;
                }

                case BREAK_BEAM_INTERMEDIATE: {
                    MotorColectarePID.targetValue -= 100;
                    MotorColectarePID.maxOutput =1;
                    break;
                }

                case DRAW_MOTOR_MID_1:
                {
                    MotorColectarePID.targetValue = 380;
                    MotorColectarePID.maxOutput = VLT2[NrConAuto];
                    break;
                }
                case DRAW_MOTOR_MID_2: {
                    MotorColectarePID.targetValue = 400;
                    MotorColectarePID.maxOutput = 0.4;
                    break;
                }

                case PulaMea:
                {
                    MotorColectarePID.targetValue = 600;
                    MotorColectarePID.maxOutput = 0.1;
                    break;
                }
                case RETRACTED_0:
                {
                    MotorColectarePID.targetValue = -10;
                    MotorColectarePID.maxOutput = 1;
                    break;
                }
                case EXTENDED_SOUTH_DREAPTA_HAZ_2:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = Extendo5_1Haz2DreaptaSouthPositions[NrConAuto];
                    break;
                }
                case MID_CU_INTF_IN_PULA_MEA:
                {
                    MotorColectarePID.maxOutput =1;
                    MotorColectarePID.targetValue = ExtendoHaz[NrConAuto] - 150;
                    break;
                }
                case EXTENDED_1050:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = extendedDrive;
                    break;
                }
                case EXTENDED_DRIVE:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = extendedDriveMax;
                    break;
                }
                case EXTENDED:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 700;
                    break;
                }
                case EXTENDED_SOUTH:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = ExtendoCyclingSouthPositions[NrConAuto];
                    break;
                }
                case EXTENDED_SOUTH_SIGUR:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = Extendo5_1SouthPositions[NrConAuto] - 100;
                    break;
                }
                case HALF_WAY:
                {
                    MotorColectarePID.maxOutput = 0.4;
                    MotorColectarePID.targetValue = 325;
                    break;
                }
                case THREE_WAY:
                {
                    MotorColectarePID.maxOutput = 0.4;
                    MotorColectarePID.targetValue = 432;
                    break;
                }
                case EXTENDED_SOUTH_LEFT:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = Extendo5_1SouthLeftPositions[NrConAuto];
                    break;
                }
                case SA_VA_DAU_LA_MUIE:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 300;
                    break;
                }
                case EXTENDED_COMMONHIGHINTERFERENCE:
                {
                    MotorColectarePID.maxOutput = 0.5;
                    switch (NrConAuto) {
                        case 1: {
                            MotorColectarePID.targetValue = 170 - 50;
                            break;
                        }
                        case 2: {
                            MotorColectarePID.targetValue = 184- 50;
                            break;
                        }
                        case 3: {
                            MotorColectarePID.targetValue = 184- 50;
                            break;
                        }
                        case 4: {
                            MotorColectarePID.targetValue = 184-50;
                            break;
                        }
                        case 5: {
                            MotorColectarePID.targetValue = 337-50;
                            break;
                        }
                    }
                    break;
                }
                case EXTENDED_5_1:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = ExtendoHaz[NrConAuto];
                    break;
                }
                case EXTENDED_5_1_LEFT:
            {
                MotorColectarePID.maxOutput = 1;
                MotorColectarePID.targetValue = ExtendoHazLeft[NrConAuto];
                break;
            }
                case EXTENDED_10_1:
                {
                    MotorColectarePID.maxOutput = 1;
                    switch (NrConAuto)
                    {
                        case 1:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_1;
                            break;
                        }
                        case 2:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_2;
                            break;
                        }
                        case 3:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_3;
                            break;
                        }
                        case 4:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_4;
                            break;
                        }
                        case 5:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_5;
                            break;
                        }
                        case 6:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_6;
                            break;
                        }
                        case 7:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_7;
                            break;
                        }
                        case 8:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_8;
                            break;
                        }
                        case 9:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_9;
                            break;
                        }
                        case 10:
                        {
                            MotorColectarePID.targetValue = extended10_1Autonomy_10;
                            break;
                        }
                    }
                    break;
                }
                case EXTENDED_10_1_SOUTH:
                {
                    MotorColectarePID.maxOutput = 1;
                    switch (NrConAuto)
                    {
                        case 1:
                        {
                            MotorColectarePID.targetValue = 680;
                            break;
                        }
                        case 2:
                        {
                            MotorColectarePID.targetValue = 680;
                            break;
                        }
                        case 3:
                        {
                            MotorColectarePID.targetValue = 680;
                            break;
                        }
                        case 4:
                        {
                            MotorColectarePID.targetValue = 680;
                            break;
                        }
                        case 5:
                        {
                            MotorColectarePID.targetValue = 680;
                            break;
                        }
                    }
                    break;
                }
                case EXTENDED_FAST:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = extendedPosition;
                    break;
                }
                case EXTENDED_2050:
                {
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 497;
                    break;
                }
                case Colectare_4bar:{
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 50;
                }
                case Colectare_siguranta:{
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 150;
                }

                case Colectare_close_claw:{
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 250;
                }

                case Colectare_liftsanki:{
                    MotorColectarePID.maxOutput = 1;
                    MotorColectarePID.targetValue = 350;
                }

            }
        }
        PreviousStatus = CurrentStatus;
    }
}

package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.INTERMEDIARY;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.Current;
@Config
public class Angle4BarController {

    public enum angle4BarStatus
    {
        INIT,
        VERTICAL,
        COLLECT_CONES,
        PLACE_AUTO,
        RAISED,
        PLACE,
        LIL_FRONT,
        LIL_RAISED,
        PLACE_LOW,
        LIL_PLACE,
        INCREMENT,
        PLACE_FIFTH_GROUND,
    }
    public static angle4BarStatus CurrentStatus = angle4BarStatus.INIT,  PreviousStatus = angle4BarStatus.INIT;
    public static double pozLilFront=0.45 , pozVertical = 0.455,pozPlaceFifthGround = 0.33,  pozCollectCones = 0.86 , pozRaised = 0.30 ,pozPlace = 0.54 , pozPlaceAuto = 0.55 , pozLilRaised = 0.39 , pozPlaceLow = 0.21,pozLilPlace = 0.5;

    public void update(RobotMap Robotel)
    {

        if (PreviousStatus != CurrentStatus || CurrentStatus==angle4BarStatus.VERTICAL)
        {
            switch (CurrentStatus) {
                case VERTICAL: {
                    Robotel.angle4Bar.setPosition(pozVertical);
                    break;
                }
                case PLACE_FIFTH_GROUND:
                {
                    Robotel.angle4Bar.setPosition(pozPlaceFifthGround);
                    break;
                }
                case LIL_FRONT: {
                    Robotel.angle4Bar.setPosition(pozLilFront);
                    break;
                }
                case LIL_PLACE: {
                    Robotel.angle4Bar.setPosition(pozLilPlace);
                    break;
                }
                case COLLECT_CONES: {
                    Robotel.angle4Bar.setPosition(pozCollectCones);
                    break;
                }
                case RAISED: {
                    Robotel.angle4Bar.setPosition(pozRaised);
                    break;
                }
                case PLACE: {
                    Robotel.angle4Bar.setPosition(pozPlace);
                    break;
                }
                case PLACE_AUTO:
                {
                    Robotel.angle4Bar.setPosition(pozPlaceAuto);
                    break;
                }
                case LIL_RAISED: {
                    Robotel.angle4Bar.setPosition(pozLilRaised);
                    break;
                }
                case PLACE_LOW: {
                    Robotel.angle4Bar.setPosition(pozPlaceLow);
                    break;
                }
                case INCREMENT: {
                    CurrentStatus = angle4BarStatus.PLACE_LOW;
                    break;
                }

            }
        }
            PreviousStatus = CurrentStatus;

    }
}

package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.BiggerController.biggerControllerStatus.COLLECT_RAPID_FIRE_INTER;
import static org.firstinspires.ftc.teamcode.drive.OGCode.BiggerController.biggerControllerStatus.COLLECT_RAPID_FIRE_INTER2;
import static org.firstinspires.ftc.teamcode.drive.OGCode.BiggerController.biggerControllerStatus.NOTHING;

import com.qualcomm.robotcore.util.ElapsedTime;

public class BiggerController {
    public enum biggerControllerStatus
    {
        NOTHING,
        COLLECT_RAPID_FIRE,
        COLLECT_RAPID_FIRE_INTER,
        COLLECT_RAPID_FIRE_INTER2,
    }
    biggerControllerStatus CurrentStatus = NOTHING, PreviousStatus = NOTHING;
    ElapsedTime timerCOLLECT_RAPID_FIRE2 = new ElapsedTime() ,timerCOLLECT_RAPID_FIRE1 = new ElapsedTime();
    public void update(RobotController robotController, CloseClawController closeClawController, MotorColectareController motorColectareController)
    {
        if (CurrentStatus!=PreviousStatus || CurrentStatus == COLLECT_RAPID_FIRE_INTER || CurrentStatus == COLLECT_RAPID_FIRE_INTER2)
        {
            switch (CurrentStatus)
            {
                case COLLECT_RAPID_FIRE:
                {
                    timerCOLLECT_RAPID_FIRE1.reset();

                    robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;

                    CurrentStatus = COLLECT_RAPID_FIRE_INTER;
                    break;
                }
                case COLLECT_RAPID_FIRE_INTER:
                {
                    if (timerCOLLECT_RAPID_FIRE1.seconds()>2)
                    {
                         timerCOLLECT_RAPID_FIRE2.reset();
                         closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                         CurrentStatus = COLLECT_RAPID_FIRE_INTER2;
                    }
                    break;
                }
                case COLLECT_RAPID_FIRE_INTER2:
                {
                    if (timerCOLLECT_RAPID_FIRE2.seconds()>0.5)
                    {
                        robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE;
                        CurrentStatus = NOTHING;
                    }
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}

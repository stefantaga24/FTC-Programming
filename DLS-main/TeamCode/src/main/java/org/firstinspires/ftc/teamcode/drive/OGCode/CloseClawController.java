package org.firstinspires.ftc.teamcode.drive.OGCode;

import org.checkerframework.checker.units.qual.Current;

public class CloseClawController {

    public enum closeClawStatus
    {
        INIT,
        CLOSED,
        OPEN,
        OPEN_COLLECT,
        OPEN_CLAW_BIG,
        OPEN_CLAW_SMALL,
    }
    public static closeClawStatus CurrentStatus = closeClawStatus.INIT,  PreviousStatus = closeClawStatus.INIT;
    double pozOpenClaw = 0.875, pozCloseClaw = 0.74 ,pozOpenClawCollect = 0.89 , pozOpenClawBig = 0.9,pozOpenClawSmall = 0.83;

    public void update(RobotMap Robotel)
    {
        if (PreviousStatus!= CurrentStatus)
        {
            switch(CurrentStatus)
            {
                case CLOSED:
                {
                    Robotel.closeClaw.setPosition(pozCloseClaw);
                    break;
                }
                case OPEN:
                {
                    Robotel.closeClaw.setPosition(pozOpenClaw);
                    break;
                }
                case OPEN_COLLECT:
                {
                    Robotel.closeClaw.setPosition(pozOpenClawCollect);
                    break;
                }
                case OPEN_CLAW_BIG:
                {
                    Robotel.closeClaw.setPosition(pozOpenClawBig);
                    break;
                }
                case OPEN_CLAW_SMALL:
                {
                    Robotel.closeClaw.setPosition(pozOpenClawSmall);
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}

package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * RED alliance: drive to the parking corner, no shooting.
 * Start: back of the robot touching the RED alliance wall, centered on the
 * middle tile seam of that wall, front facing into the field.
 * See ParkAutoBase for the plan, settings, and sensor names.
 */
@Autonomous(name = "Red Park Auto", group = "Pedro")
public class RedParkAuto extends ParkAutoBase {
    @Override
    protected boolean isRed() {
        return true;
    }
}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Context;

@TeleOp
public class ResetRuntime extends OpMode {
    @Override
    public void init() {
        Context.runtime = null;
        Context.reset();
    }

    @Override
    public void loop() {}
}

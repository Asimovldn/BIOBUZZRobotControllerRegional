package org.firstinspires.ftc.teamcode.util;

public class FFController
{
    private final double kv;
    private final double ka;
    private final double ks;

    public FFController(double kv, double ka, double ks)
    {
        this.kv = kv;
        this.ka = ka;
        this.ks = ks;
    }

    public double calculate(double v, double a)
    {
        return kv*v + ka*a + ks*Math.signum(v);
    }

}

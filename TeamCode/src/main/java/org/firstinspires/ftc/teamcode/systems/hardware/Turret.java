package org.firstinspires.ftc.teamcode.systems.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.FFController;
import org.firstinspires.ftc.teamcode.util.Util;

public class Turret extends Mechanism
{
    @Config
    public static class Tune{
        //TODO calibrar o feedforward (mas primeiro olha embaixo)
        public static double kv = 0;
        public static double ka = 0;
        public static double ks = 0;

        //TODO calibrar o sensor, ler qual voltagem ele mostra nos limites e colocara qui
        public static double MIN_SENSOR_VOLTAGE = 0;
        public static double MAX_SENSOR_VOLTAGE = 0;
        public static double MIN_ANGLE = 0;
        public static double MAX_ANGLE = 0;
    }

    private static final String SERVO_NAME = "turret";
    private static final String SENSOR_NAME = "turret_sensor";

    CRServo crServo;

    AnalogInput servoSensor;

    private double lastNs;
    private double deltaNs = 0;
    private double lastPos;
    private double deltaPos;

    public Turret(Context runtime)
    {
        passInRuntime(runtime);
        HardwareMap hw = runtime.hardwareMap;

        crServo = hw.get(CRServoImpl.class, SERVO_NAME);
        servoSensor = hw.get(AnalogInput.class, SENSOR_NAME);

        crServo.setDirection(DcMotorSimple.Direction.FORWARD);
        
        lastNs = System.nanoTime();
        lastPos = getTurretPosition();
    }

    /** Converts sensor position to deg */
    private double getTurretPosition()
    {
        double t = (servoSensor.getVoltage() - Tune.MIN_SENSOR_VOLTAGE) / (Tune.MAX_SENSOR_VOLTAGE - Tune.MIN_SENSOR_VOLTAGE);
        return Util.wrapDeg(Util.lerp(Tune.MIN_ANGLE, Tune.MAX_ANGLE, t));
    }

    private double getTurretVelocity()
    {
        return deltaNs == 0.0 ? 0.0 : deltaPos / (deltaNs * 10e-9);
    }

    private double calculateServoPower(double vel, double accel)
    {
        FFController ff = new FFController(Tune.kv, Tune.ka, Tune.ks);
        return ff.calculate(vel, accel);
    }

    public void runServo(double vel)
    {
        crServo.setPower(calculateServoPower(vel,0));
    }

    @Override
    public void periodic() {
        deltaNs = System.nanoTime() - lastNs;
        lastNs = System.nanoTime();

        deltaPos = Util.wrapDeg(getTurretPosition() - lastPos);
        lastPos = getTurretPosition();
    }
}

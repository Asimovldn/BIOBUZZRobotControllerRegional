package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.systems.hardware.Intake;
import org.firstinspires.ftc.teamcode.systems.hardware.Shooter;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.LazyDrive;

import java.util.Set;

public class RobotOpMode extends OpMode {
    public Follower follower;
    public Turret turret;
    public Shooter shooter;
    public Intake intake;
    public Context.Alliance alliance;

    /** DO NOT EDIT, runtime is managed by RobotOpMode Context. Do not override */
    public Context runtime;

    public LazyDrive drivetrain;

    /** Override based on your OpMode, gets the Alliance from here */
    public Context.Alliance allianceForOpMode() {
        return Context.Alliance.RED;
    }

    public String locationForOpMode() { return "Audience"; }

    @Override
    public void init() {
        if (Context.runtime == null) {
            runtime = Context.setRuntime(new Context());
        } else {
            runtime = Context.runtime;
        }

        runtime.hardwareMap = hardwareMap;
        runtime.location = locationForOpMode();

        setAlliance(allianceForOpMode());

        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret(runtime);

        drivetrain = new LazyDrive();

        follower = getFollower();

        for (Mechanism mech : getMechanisms()) {
            mech.passInRuntime(runtime);
            mech.setup();
        }

        create();
    }

    /** Equivalent to init() */
    public void create() {

    }

    public void setup() {

    }

    @Override
    public void start() {
        setup();
        follower.update();
    }

    public Turret getTurret() {
        return turret;
    }

    public Shooter getShooter() { return shooter; }

    public Intake getIntake() { return intake; }

    public Follower getFollower() {
        if (follower == null) {
            Pose carried = runtime.pose;

            follower = Constants.create(runtime.hardwareMap);
            if (carried != null) follower.setPose(carried);

            runtime.update(null, null, null, follower);
        }

        return follower;
    }

    public void setAlliance(Context.Alliance alliance) {
        this.alliance = alliance;
        runtime.GlobalAlliance = alliance;
    }

    public Context.Alliance getAlliance() {
        return alliance;
    }

    public void drive(Gamepad gmp) {
        ManualDrive.driveOrHold(getFollower(), -gmp.left_stick_y, -gmp.left_stick_x, -gmp.right_stick_x);
    }

    public void drive(DrivePowers pwrs) {
        drivetrain.drive(getFollower(), pwrs);
    }

    public Command autoDrive(Gamepad gmp) {
        return Command.build()
                .setExecute(() -> drive(gmp))
                .setDone(() -> drivetrain.isDriving())
                .requiring(this);
    }

    public void fieldCentric(Gamepad gmp) {
        drivetrain.fieldCentric(getFollower(), -gmp.left_stick_y, -gmp.left_stick_x, -gmp.right_stick_x);
    }

    @Override
    public void loop() {
        updateMechanisms();
        periodic();
        follower.update();

        runtime.update(alliance, null, follower.pose(), follower);
    }

    /** Your Code Loop, called as {@code loop()}. */
    public void periodic() {

    }

    protected void updateMechanisms() {
        for (Mechanism mech : getMechanisms()) {
            mech.periodic();
        }
    }

    @NonNull
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, shooter, turret);
    }

    /** Equivalent to super, use this to get access to ONLY {@code robotOpMode} methods */
    public RobotOpMode bot() { return this; }
}

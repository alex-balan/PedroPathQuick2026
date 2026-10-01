package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Example Auto", group = "Examples")
//@Disabled
public class ExampleAuto extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer;

    private Path startToScore;
    private Path park;

    private int pathState;

    // Poses
    private final Pose startPose = new Pose(24, 24, 0);
    private final Pose scorePose = new Pose(48, 48, Math.toRadians(90));
    private final Pose parkPose = new Pose(72, 48, Math.toRadians(90));

    /**
     * Build the paths for the autonomous routine.
     */
    public void buildPaths() {
        startToScore = line(startPose, scorePose).constant(Math.toRadians(90));
        park = line(scorePose, parkPose).constant(Math.toRadians(90));
    }

    /**
     * State machine to manage path following execution.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.follow(startToScore);
                setPathState(1);
                break;
            case 1:
                if (follower.atParametricEnd()) {
                    follower.hold(scorePose);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.seconds() >= 5.0) {
                    follower.follow(park);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.atParametricEnd()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.reset();
    }

    @Override
    public void init() {
        pathState = 0;
        pathTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        follower.setPose(startPose);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}

package frc.team3128.commands;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

public class CmdSentience extends CommandBase{
    
    private Swerve swerve;
    private static SwerveAutoBuilder builder;
    private static HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private static HashMap<String, PathPoint> points = new HashMap<String, PathPoint>();
    private Pose2d scoringPoint;
    private static PathConstraints constraints = new PathConstraints(maxAngularVelocity, maxAcceleration);
    PathPlannerTrajectory scoreTraj;

    public CmdSentience(int scoringPoint) {
        this(SCORES[scoringPoint - 1]);
    }

    public CmdSentience(Pose2d scoringPoint) {
        this.scoringPoint = scoringPoint;
        swerve = Swerve.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        builder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            swerveKinematics,
            new PIDConstants(translationKP,translationKI,translationKD),
            new PIDConstants(rotationKP,rotationKI,rotationKD),
            swerve::setModuleStates,
            eventMap,
            true,
            swerve
        );
        
        initPointMap();
        scoreTraj = PathPlanner.generatePath(constraints, calculatePath());
    }

    public void initPointMap(){
        points.put("start", new PathPoint(swerve.getPose().getTranslation(), swerve.getRotation2d()));
        points.put("topBnd", new PathPoint(new Translation2d(4.85, 4.60), new Rotation2d(180), new Rotation2d(180)));
        points.put("botBnd", new PathPoint(new Translation2d(4.85, 0.90), new Rotation2d(180), new Rotation2d(180)));
        points.put("topInt", new PathPoint(new Translation2d(2.50, 4.60), new Rotation2d(135), new Rotation2d(180)));
        points.put("botInt", new PathPoint(new Translation2d(2.50, 0.90), new Rotation2d(-135), new Rotation2d(180)));
        points.put("endTopAprch", new PathPoint(scoringPoint.getTranslation(), new Rotation2d(-90), new Rotation2d(180)));
        points.put("endBotAprch", new PathPoint(scoringPoint.getTranslation(), new Rotation2d(90), new Rotation2d(180)));
    }

    public ArrayList<PathPoint> calculatePath (){
        ArrayList<PathPoint> path = new ArrayList<PathPoint>();
        boolean topApproach = scoringPoint.getY() > 3.00;
        
        if(topApproach) {
            path.add(0, points.get("endTopAprch"));
        }
        else path.add(0, points.get("endBotAprch"));
        
        // if(swerve.getPose().getX() >= 2.50 && swerve.getPose().getY() >= 4.60) 
        //     path.add(0, points.get("topInt"));
        // else if(swerve.getPose().getX() >= 2.50 || swerve.getPose().getY() <= 0.9) 
        //     path.add(0, points.get("botInt"));

        if(swerve.getPose().getX() >= 2.50) path.add(0, topApproach ? points.get("topInt") : points.get("botInt"));
        if(swerve.getPose().getX() > 4.85) path.add(0, topApproach ? points.get("topBnd") : points.get("botBnd"));
        path.add(0, points.get("start"));
        
        return path;
    }

    @Override
    public void execute(){
        builder.fullAuto(scoreTraj);
    }

    @Override
    public boolean isFinished(){
        return swerve.getPose().relativeTo(scoringPoint).getTranslation().getNorm() < 0.05;
    }
    
}

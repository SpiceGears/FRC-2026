package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstats;
import frc.robot.subsystems.vision.limelight.AprilTagVisionSubsystem;

public class ShooterVisionAid {
    InterpolatingDoubleTreeMap shooterMap;
    InterpolatingDoubleTreeMap hoodMap;

    public static ShooterVisionAid instance;

    static double cachedKey;

    AprilTagVisionSubsystem vision;

    Supplier<Pose2d> robotPoseSupplier;

    final double MINIMUM_DISTANCE = 0;
    final double MAXIMUM_DISTANCE = ShooterConstats.MAX_INTERPOLATED_RANGE.in(Meters);

    public ShooterVisionAid(Supplier<Pose2d> robotPoseSupplier) 
    {
        if (instance != null) {
            return;
        }

        this.robotPoseSupplier = robotPoseSupplier;

        hoodMap = new InterpolatingDoubleTreeMap();
        shooterMap = new InterpolatingDoubleTreeMap();

        vision = AprilTagVisionSubsystem.instance;


        instance = this;
    }


    public void addHoodPositionKey(double key, double positionMM) 
    {
        hoodMap.put(key, positionMM);
    }

    public void addShooterRPMKey(double key, double rpm) 
    {
        shooterMap.put(key, rpm);
    }


    public AngularVelocity getRPM(double key) 
    {
        return RPM.of(shooterMap.get(key));
    }

    public Distance getHoodPosition(double key) 
    {
        return Millimeter.of(hoodMap.get(key));
    }

    public double getKeyByDistance(Distance distance) 
    {
        double distMeters = distance.in(Meters);
        double scaledKey = (distMeters - MINIMUM_DISTANCE) / (MAXIMUM_DISTANCE - MINIMUM_DISTANCE);
        scaledKey = MathUtil.clamp(scaledKey, 0, 1);
        return scaledKey;
    }

    public Distance getDistanceToTarget(Pose3d target) 
    {
        return Meters.of(
            target.toPose2d().getTranslation().
            getDistance(Pose2d.kZero.getTranslation()));
    }


    public Optional<Pose3d> getNearestTarget() 
    {
        Optional<List<Pair<Double, Pose3d>>> results = vision.getTargetPoses();
        if (results.isEmpty()) return Optional.empty(); // no useful vision data, return nothing

        List<Pair<Double, Pose3d>> targets = results.get();

        List<Pose3d> filteredTags = new LinkedList<>();
        for (Pair<Double, Pose3d> tag : targets) 
        {
            if (AutoConstants.TAGS_OF_INTEREST_HUB.contains(tag.getFirst()))
            { filteredTags.add(tag.getSecond()); }
        }

        Pose3d nearestTag = Pose3d.kZero.plus(new Transform3d(999999, 999999, 999999, new Rotation3d(0,0,0)));
        if (filteredTags.size() <= 0) return Optional.empty(); // protect in case all detected tags are not our point of interest
        for (Pose3d tagPose : filteredTags) 
        {
            if (getDistanceToTarget(nearestTag).gt(getDistanceToTarget(tagPose)))
            nearestTag = tagPose;
        }
        return Optional.of(nearestTag);
    }

    public Optional<Double> getCurrentlyApplicableKey() 
    {
        Optional<List<Pair<Double, Pose3d>>> results = vision.getTargetPoses();
        if (results.isEmpty()) return Optional.empty(); // no useful vision data, return nothing

        List<Pair<Double, Pose3d>> targets = results.get();

        List<Pose3d> filteredTags = new LinkedList<>();
        for (Pair<Double, Pose3d> tag : targets) 
        {
            if (AutoConstants.TAGS_OF_INTEREST_HUB.contains(tag.getFirst()))
            { filteredTags.add(tag.getSecond()); }
        }

        Pose3d nearestTag = Pose3d.kZero.plus(new Transform3d(999999, 999999, 999999, new Rotation3d(0,0,0)));
        if (filteredTags.size() <= 0) return Optional.empty(); // protect in case all detected tags are not our point of interest
        for (Pose3d tagPose : filteredTags) 
        {
            if (getDistanceToTarget(nearestTag).gt(getDistanceToTarget(tagPose)))
            nearestTag = tagPose;
        }

        Distance distanceToNearest = getDistanceToTarget(nearestTag);
        return Optional.of(getKeyByDistance(distanceToNearest));
    }

    public double getCurrentOrCachedKey() 
    {
        Optional<Double> current = getCurrentlyApplicableKey();
        if (current.isPresent()) {
            cachedKey = current.get();
        }
        return cachedKey;
    } 




    public void telemetry() 
    {
        double target_count = 0;
        SmartDashboard.putNumber("SVAS/Cached Key", cachedKey);
        SmartDashboard.putNumber("SVAS/Nearest Target Distance Meters", (getNearestTarget().isPresent()) ? (getDistanceToTarget(getNearestTarget().get())).in(Meters) : (-1));
        SmartDashboard.putNumber("SVAS/Current Hood setting milimeters", getHoodPosition(cachedKey).in(Millimeters));
        SmartDashboard.putNumber("SVAS/Current Adaptable RPM setting", getRPM(cachedKey).in(RPM));

    }

}

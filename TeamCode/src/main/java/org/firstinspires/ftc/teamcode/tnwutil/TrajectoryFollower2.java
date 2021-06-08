package org.firstinspires.ftc.teamcode.tnwutil;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

import kotlin.Metadata;
import kotlin.collections.CollectionsKt;
import kotlin.comparisons.ComparisonsKt;
import kotlin.jvm.JvmOverloads;
import kotlin.jvm.internal.Intrinsics;

@Metadata(
        mv = {1, 4, 2},
        bv = {1, 0, 3},
        k = 1,
        d1 = {"\u0000H\n\u0002\u0018\u0002\n\u0002\u0010\u0000\n\u0000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u0006\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0010\u000b\n\u0002\b\n\n\u0002\u0010!\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0007\n\u0002\u0010\u0002\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0002\b\u0005\b&\u0018\u00002\u00020\u0001B%\b\u0007\u0012\b\b\u0002\u0010\u0002\u001a\u00020\u0003\u0012\b\b\u0002\u0010\u0004\u001a\u00020\u0005\u0012\b\b\u0002\u0010\u0006\u001a\u00020\u0007¢\u0006\u0002\u0010\bJ\u0006\u0010\u001e\u001a\u00020\u0005J\u0010\u0010\u001f\u001a\u00020 2\u0006\u0010\u0019\u001a\u00020\u0018H\u0016J\b\u0010!\u001a\u00020\nH\u0002J\u001a\u0010\"\u001a\u00020#2\u0006\u0010$\u001a\u00020\u00032\b\u0010%\u001a\u0004\u0018\u00010\u0003H$J\u0006\u0010&\u001a\u00020\nJ\u001c\u0010'\u001a\u00020#2\u0006\u0010$\u001a\u00020\u00032\n\b\u0002\u0010%\u001a\u0004\u0018\u00010\u0003H\u0007R\u000e\u0010\t\u001a\u00020\nX\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u0002\u001a\u00020\u0003X\u0082\u0004¢\u0006\u0002\n\u0000R\u0014\u0010\u0006\u001a\u00020\u0007X\u0084\u0004¢\u0006\b\n\u0000\u001a\u0004\b\u000b\u0010\fR\u000e\u0010\r\u001a\u00020\nX\u0082\u000e¢\u0006\u0002\n\u0000R\"\u0010\u000f\u001a\u00020\u00032\u0006\u0010\u000e\u001a\u00020\u0003@dX¦\u000e¢\u0006\f\u001a\u0004\b\u0010\u0010\u0011\"\u0004\b\u0012\u0010\u0013R\u0014\u0010\u0014\u001a\b\u0012\u0004\u0012\u00020\u00160\u0015X\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u0017\u001a\u00020\u0005X\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u0004\u001a\u00020\u0005X\u0082\u0004¢\u0006\u0002\n\u0000R$\u0010\u0019\u001a\u00020\u00182\u0006\u0010\u000e\u001a\u00020\u0018@DX\u0086.¢\u0006\u000e\n\u0000\u001a\u0004\b\u001a\u0010\u001b\"\u0004\b\u001c\u0010\u001d¨\u0006("},
        d2 = {"Lorg/firstinspires/ftc/teamcode/tnwutil/TrajectoryFollower;", "", "admissibleError", "Lcom/acmerobotics/roadrunner/geometry/Pose2d;", "timeout", "", "clock", "Lcom/acmerobotics/roadrunner/util/NanoClock;", "(Lcom/acmerobotics/roadrunner/geometry/Pose2d;DLcom/acmerobotics/roadrunner/util/NanoClock;)V", "admissible", "", "getClock", "()Lcom/acmerobotics/roadrunner/util/NanoClock;", "executedFinalUpdate", "<set-?>", "lastError", "getLastError", "()Lcom/acmerobotics/roadrunner/geometry/Pose2d;", "setLastError", "(Lcom/acmerobotics/roadrunner/geometry/Pose2d;)V", "remainingMarkers", "", "Lcom/acmerobotics/roadrunner/trajectory/TrajectoryMarker;", "startTimestamp", "Lcom/acmerobotics/roadrunner/trajectory/Trajectory;", "trajectory", "getTrajectory", "()Lcom/acmerobotics/roadrunner/trajectory/Trajectory;", "setTrajectory", "(Lcom/acmerobotics/roadrunner/trajectory/Trajectory;)V", "elapsedTime", "followTrajectory", "", "internalIsFollowing", "internalUpdate", "Lcom/acmerobotics/roadrunner/drive/DriveSignal;", "currentPose", "currentRobotVel", "isFollowing", "update", "tnw-UltimateGoal_.TeamCode"}
)
public abstract class TrajectoryFollower2 {
    private double startTimestamp;
    private boolean admissible;
    private List remainingMarkers;
    private boolean executedFinalUpdate;
    protected Trajectory trajectory;
    private final Pose2d admissibleError;
    private final double timeout;
    @NotNull
    private final NanoClock clock;

    @NotNull
    public final Trajectory getTrajectory() {
        Trajectory var10000 = this.trajectory;
        if (var10000 == null) {
            Intrinsics.throwUninitializedPropertyAccessException("trajectory");
        }

        return var10000;
    }

    protected final void setTrajectory(@NotNull Trajectory var1) {
        //Intrinsics.checkNotNullParameter(var1, "<set-?>");
        this.trajectory = var1;
    }

    @NotNull
    public abstract Pose2d getLastError();

    protected abstract void setLastError(@NotNull Pose2d var1);

    public void followTrajectory(@NotNull Trajectory trajectory) {
        //Intrinsics.checkNotNullParameter(trajectory, "trajectory");
        this.startTimestamp = this.clock.seconds();
        this.trajectory = trajectory;
        this.admissible = false;
        this.remainingMarkers.clear();
        this.remainingMarkers.addAll((Collection)trajectory.getMarkers());
        List $this$sortBy$iv = this.remainingMarkers;
        int $i$f$sortBy = 0;
        if ($this$sortBy$iv.size() > 1) {
            boolean var4 = false;
            CollectionsKt.sortWith($this$sortBy$iv, (Comparator)(new TrajectoryFollower$followTrajectory$$inlined$sortBy$1()));
        }

        this.executedFinalUpdate = false;
    }

    private final boolean internalIsFollowing() {
        Trajectory var10000 = this.trajectory;
        if (var10000 == null) {
            Intrinsics.throwUninitializedPropertyAccessException("trajectory");
        }

        double timeRemaining = var10000.duration() - this.elapsedTime();
        return timeRemaining > (double)0 || !this.admissible && timeRemaining > -this.timeout;
    }

    public final boolean isFollowing() {
        return !this.executedFinalUpdate || this.internalIsFollowing();
    }

    public final double elapsedTime() {
        return this.clock.seconds() - this.startTimestamp;
    }

    @JvmOverloads
    @NotNull
    public final DriveSignal update(@NotNull Pose2d currentPose, @Nullable Pose2d currentRobotVel) {
        //Intrinsics.checkNotNullParameter(currentPose, "currentPose");

        while(this.remainingMarkers.size() > 0 && this.elapsedTime() > ((TrajectoryMarker)this.remainingMarkers.get(0)).getTime()) {
            ((TrajectoryMarker)this.remainingMarkers.remove(0)).getCallback().onMarkerReached();
        }

        Trajectory var10000 = this.trajectory;
        if (var10000 == null) {
            Intrinsics.throwUninitializedPropertyAccessException("trajectory");
        }

        boolean var10001;
        label39: {
            Pose2d trajEndError = var10000.end().minus(currentPose);
            double var4 = trajEndError.getX();
            boolean var6 = false;
            if (Math.abs(var4) < this.admissibleError.getX()) {
                var4 = trajEndError.getY();
                var6 = false;
                if (Math.abs(var4) < this.admissibleError.getY()) {
                    var4 = Angle.normDelta(trajEndError.getHeading());
                    var6 = false;
                    if (Math.abs(var4) < this.admissibleError.getHeading()) {
                        var10001 = true;
                        break label39;
                    }
                }
            }

            var10001 = false;
        }

        this.admissible = var10001;
        DriveSignal var8;
        if (!this.internalIsFollowing() && !this.executedFinalUpdate) {
            Iterator var5 = this.remainingMarkers.iterator();

            while(var5.hasNext()) {
                TrajectoryMarker marker = (TrajectoryMarker)var5.next();
                marker.getCallback().onMarkerReached();
            }

            this.remainingMarkers.clear();
            this.executedFinalUpdate = true;
            var8 = new DriveSignal();
        } else {
            var8 = this.internalUpdate(currentPose, currentRobotVel);
        }

        return var8;
    }

    // $FF: synthetic method
    public static DriveSignal update$default(TrajectoryFollower2 var0, Pose2d var1, Pose2d var2, int var3, Object var4) {
        if (var4 != null) {
            throw new UnsupportedOperationException("Super calls with default arguments not supported in this target, function: update");
        } else {
            if ((var3 & 2) != 0) {
                var2 = (Pose2d)null;
            }

            return var0.update(var1, var2);
        }
    }

    @JvmOverloads
    @NotNull
    public final DriveSignal update(@NotNull Pose2d currentPose) {
        return update$default(this, currentPose, (Pose2d)null, 2, (Object)null);
    }

    @NotNull
    protected abstract DriveSignal internalUpdate(@NotNull Pose2d var1, @Nullable Pose2d var2);

    @NotNull
    protected final NanoClock getClock() {
        return this.clock;
    }

    @JvmOverloads
    public TrajectoryFollower2(@NotNull Pose2d admissibleError, double timeout, @NotNull NanoClock clock) {
        //Intrinsics.checkNotNullParameter(admissibleError, "admissibleError");
        //Intrinsics.checkNotNullParameter(clock, "clock");
        //super();
        this.admissibleError = admissibleError;
        this.timeout = timeout;
        this.clock = clock;
        boolean var5 = false;
        this.remainingMarkers = (new ArrayList());
    }

    // $FF: synthetic method
//    public TrajectoryFollower2(Pose2d var1, double var2, NanoClock var4, int var5) {
//        if ((var5 & 1) != 0) {
//            var1 = new Pose2d(0.0D, 0.0D, 0.0D);
//        }
//
//        if ((var5 & 2) != 0) {
//            var2 = 0.0D;
//        }
//
//        if ((var5 & 4) != 0) {
//            var4 = NanoClock.Companion.system();
//        }
//
//        this(var1, var2, var4);
//    }

    @JvmOverloads
    public TrajectoryFollower2(@NotNull Pose2d admissibleError, double timeout) {
        this(admissibleError, timeout, (NanoClock)null);
    }

    @JvmOverloads
    public TrajectoryFollower2(@NotNull Pose2d admissibleError) {
        this(admissibleError, 0.0D, (NanoClock)null);
    }

    @JvmOverloads
    public TrajectoryFollower2() {
        this((Pose2d)null, 0.0D, (NanoClock)null);
    }
}

@Metadata(
        mv = {1, 4, 2},
        bv = {1, 0, 3},
        k = 3,
        d1 = {"\u0000\n\n\u0000\n\u0002\u0010\b\n\u0002\b\u0007\u0010\u0000\u001a\u00020\u0001\"\u0004\b\u0000\u0010\u00022\u000e\u0010\u0003\u001a\n \u0004*\u0004\u0018\u0001H\u0002H\u00022\u000e\u0010\u0005\u001a\n \u0004*\u0004\u0018\u0001H\u0002H\u0002H\n¢\u0006\u0004\b\u0006\u0010\u0007¨\u0006\b"},
        d2 = {"<anonymous>", "", "T", "a", "kotlin.jvm.PlatformType", "b", "compare", "(Ljava/lang/Object;Ljava/lang/Object;)I", "kotlin/comparisons/ComparisonsKt__ComparisonsKt$compareBy$2"}
)
final class TrajectoryFollower$followTrajectory$$inlined$sortBy$1 implements Comparator {
    public final int compare(Object a, Object b) {
        boolean var3 = false;
        TrajectoryMarker it = (TrajectoryMarker)a;
        int var5 = 0;
        Comparable<Double> var10000 = (Comparable<Double>)it.getTime();
        it = (TrajectoryMarker)b;
        Comparable<Double> var6 = var10000;
        var5 = 0;
        Double var7 = it.getTime();
        return ComparisonsKt.compareValues(var6, (Comparable<Double>)var7);
    }
}

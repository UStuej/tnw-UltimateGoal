// HolonomicPIDVAFollower2.java
package org.firstinspires.ftc.teamcode.tnwutil;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import kotlin.Metadata;
import kotlin.jvm.JvmOverloads;
import kotlin.jvm.functions.Function2;
import kotlin.jvm.internal.Intrinsics;

//import kotlin.jvm.internal.DefaultConstructorMarker;

@Metadata(
        mv = {1, 4, 2},
        bv = {1, 0, 3},
        k = 1,
        d1 = {"\u0000B\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0003\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u0006\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0002\b\t\n\u0002\u0010\u0002\n\u0000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0003\u0018\u00002\u00020\u0001B=\b\u0007\u0012\u0006\u0010\u0002\u001a\u00020\u0003\u0012\u0006\u0010\u0004\u001a\u00020\u0003\u0012\u0006\u0010\u0005\u001a\u00020\u0003\u0012\b\b\u0002\u0010\u0006\u001a\u00020\u0007\u0012\b\b\u0002\u0010\b\u001a\u00020\t\u0012\b\b\u0002\u0010\n\u001a\u00020\u000b¢\u0006\u0002\u0010\fJ\u0010\u0010\u0017\u001a\u00020\u00182\u0006\u0010\u0019\u001a\u00020\u001aH\u0016J\u001a\u0010\u001b\u001a\u00020\u001c2\u0006\u0010\u001d\u001a\u00020\u00072\b\u0010\u001e\u001a\u0004\u0018\u00010\u0007H\u0014R\u000e\u0010\r\u001a\u00020\u000eX\u0082\u0004¢\u0006\u0002\n\u0000R\u000e\u0010\u000f\u001a\u00020\u000eX\u0082\u0004¢\u0006\u0002\n\u0000R$\u0010\u0011\u001a\u00020\u00072\u0006\u0010\u0010\u001a\u00020\u0007@TX\u0096\u000e¢\u0006\u000e\n\u0000\u001a\u0004\b\u0012\u0010\u0013\"\u0004\b\u0014\u0010\u0015R\u000e\u0010\u0016\u001a\u00020\u000eX\u0082\u0004¢\u0006\u0002\n\u0000¨\u0006\u001f"},
        d2 = {"Lorg/firstinspires/ftc/teamcode/tnwutil/HolonomicPIDVAFollower2;", "Lorg/firstinspires/ftc/teamcode/tnwutil/TrajectoryFollower;", "axialCoeffs", "Lcom/acmerobotics/roadrunner/control/PIDCoefficients;", "lateralCoeffs", "headingCoeffs", "admissibleError", "Lcom/acmerobotics/roadrunner/geometry/Pose2d;", "timeout", "", "clock", "Lcom/acmerobotics/roadrunner/util/NanoClock;", "(Lcom/acmerobotics/roadrunner/control/PIDCoefficients;Lcom/acmerobotics/roadrunner/control/PIDCoefficients;Lcom/acmerobotics/roadrunner/control/PIDCoefficients;Lcom/acmerobotics/roadrunner/geometry/Pose2d;DLcom/acmerobotics/roadrunner/util/NanoClock;)V", "axialController", "Lcom/acmerobotics/roadrunner/control/PIDFController;", "headingController", "<set-?>", "lastError", "getLastError", "()Lcom/acmerobotics/roadrunner/geometry/Pose2d;", "setLastError", "(Lcom/acmerobotics/roadrunner/geometry/Pose2d;)V", "lateralController", "followTrajectory", "", "trajectory", "Lcom/acmerobotics/roadrunner/trajectory/Trajectory;", "internalUpdate", "Lcom/acmerobotics/roadrunner/drive/DriveSignal;", "currentPose", "currentRobotVel", "tnw-UltimateGoal_.TeamCode"}
)
public final class HolonomicPIDVAFollower3 extends TrajectoryFollower2 {
    private final PIDFController axialController;
    private final PIDFController lateralController;
    private final PIDFController headingController;
    @NotNull
    private Pose2d lastError;

    @NotNull
    public Pose2d getLastError() {
        return this.lastError;
    }

    protected void setLastError(@NotNull Pose2d var1) {
        Intrinsics.checkNotNullParameter(var1, "<set-?>");
        this.lastError = var1;
    }

    public void followTrajectory(@NotNull Trajectory trajectory) {
        Intrinsics.checkNotNullParameter(trajectory, "trajectory");
        this.axialController.reset();
        this.lateralController.reset();
        this.headingController.reset();
        super.followTrajectory(trajectory);
    }

    @NotNull
    public DriveSignal internalUpdate(@NotNull Pose2d currentPose, @Nullable Pose2d currentRobotVel) {
        Intrinsics.checkNotNullParameter(currentPose, "currentPose");
        double t = this.elapsedTime();
        Pose2d targetPose = this.getTrajectory().get(t);
        Pose2d targetVel = this.getTrajectory().velocity(t);
        Pose2d targetAccel = this.getTrajectory().acceleration(t);
        Pose2d targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel);
        Pose2d targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel);
        Pose2d poseError = HolonomicPIDVAFollower2Kt.calculateRobotPoseError(targetPose, currentPose);
        this.axialController.setTargetPosition(poseError.getX());
        this.lateralController.setTargetPosition(poseError.getY());
        this.headingController.setTargetPosition(poseError.getHeading());
        this.axialController.setTargetVelocity(targetRobotVel.getX());
        this.lateralController.setTargetVelocity(targetRobotVel.getY());
        this.headingController.setTargetVelocity(targetRobotVel.getHeading());
        double axialCorrection = this.axialController.update(0.0D, currentRobotVel != null ? currentRobotVel.getX() : null);
        double lateralCorrection = this.lateralController.update(0.0D, currentRobotVel != null ? currentRobotVel.getY() : null);
        double headingCorrection = this.headingController.update(0.0D, currentRobotVel != null ? currentRobotVel.getHeading() : null);
        Pose2d correctedVelocity = targetRobotVel.plus(new Pose2d(axialCorrection, lateralCorrection, headingCorrection));
        this.setLastError(poseError);
        return new DriveSignal(correctedVelocity, targetRobotAccel);
    }
    @NotNull
    public DriveSignal internalUpdate(@NotNull Pose2d currentPose, @NotNull Pose2d targetPose, @NotNull Pose2d targetVel, @NotNull Pose2d targetAccel, @Nullable Pose2d currentRobotVel) {
        //Intrinsics.checkNotNullParameter(currentPose, "currentPose");
        double t = this.elapsedTime();
        Pose2d targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel);
        Pose2d targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel);
        Pose2d poseError = HolonomicPIDVAFollower2Kt.calculateRobotPoseError(targetPose, currentPose);
        this.axialController.setTargetPosition(poseError.getX());
        this.lateralController.setTargetPosition(poseError.getY());
        this.headingController.setTargetPosition(poseError.getHeading());
        this.axialController.setTargetVelocity(targetRobotVel.getX());
        this.lateralController.setTargetVelocity(targetRobotVel.getY());
        this.headingController.setTargetVelocity(targetRobotVel.getHeading());
        double axialCorrection = this.axialController.update(0.0D, currentRobotVel != null ? currentRobotVel.getX() : null);
        double lateralCorrection = this.lateralController.update(0.0D, currentRobotVel != null ? currentRobotVel.getY() : null);
        double headingCorrection = this.headingController.update(0.0D, currentRobotVel != null ? currentRobotVel.getHeading() : null);
        Pose2d correctedVelocity = targetRobotVel.plus(new Pose2d(axialCorrection, lateralCorrection, headingCorrection));
        this.setLastError(poseError);
        return new DriveSignal(correctedVelocity, targetRobotAccel);
    }

    @JvmOverloads
    public HolonomicPIDVAFollower3(@NotNull PIDCoefficients axialCoeffs, @NotNull PIDCoefficients lateralCoeffs, @NotNull PIDCoefficients headingCoeffs, @NotNull Pose2d admissibleError, double timeout, @NotNull NanoClock clock) {
        super(admissibleError, timeout, clock);
//        Intrinsics.checkNotNullParameter(axialCoeffs, "axialCoeffs");
//        Intrinsics.checkNotNullParameter(lateralCoeffs, "lateralCoeffs");
//        Intrinsics.checkNotNullParameter(headingCoeffs, "headingCoeffs");
//        Intrinsics.checkNotNullParameter(admissibleError, "admissibleError");
//        Intrinsics.checkNotNullParameter(clock, "clock");
        this.axialController = new PIDFController(axialCoeffs, 0.0D, 0.0D, 0.0D);
        this.lateralController = new PIDFController(lateralCoeffs, 0.0D, 0.0D, 0.0D);
        this.headingController = new PIDFController(headingCoeffs, 0.0D, 0.0D, 0.0D);
        this.lastError = new Pose2d(0.0D, 0.0D, 0.0D);
        this.headingController.setInputBounds(-3.141592653589793D, 3.141592653589793D);
    }

    // $FF: synthetic method
//    public HolonomicPIDVAFollower3(PIDCoefficients var1, PIDCoefficients var2, PIDCoefficients var3, Pose2d var4, double var5, NanoClock var7, int var8) {
//        if ((var8 & 8) != 0) {
//            var4 = new Pose2d(0.0D, 0.0D, 0.0D);
//        }
//
//        if ((var8 & 16) != 0) {
//            var5 = 0.0D;
//        }
//
//        if ((var8 & 32) != 0) {
//            var7 = NanoClock.Companion.system();
//        }
//
//        this(var1, var2, var3, var4, var5, var7);
//    }

    @JvmOverloads
    public HolonomicPIDVAFollower3(@NotNull PIDCoefficients axialCoeffs, @NotNull PIDCoefficients lateralCoeffs, @NotNull PIDCoefficients headingCoeffs, @NotNull Pose2d admissibleError, double timeout) {
        this(axialCoeffs, lateralCoeffs, headingCoeffs, admissibleError, timeout, (NanoClock)null);
    }

    @JvmOverloads
    public HolonomicPIDVAFollower3(@NotNull PIDCoefficients axialCoeffs, @NotNull PIDCoefficients lateralCoeffs, @NotNull PIDCoefficients headingCoeffs, @NotNull Pose2d admissibleError) {
        this(axialCoeffs, lateralCoeffs, headingCoeffs, admissibleError, 0.0D, (NanoClock)null);
    }

    @JvmOverloads
    public HolonomicPIDVAFollower3(@NotNull PIDCoefficients axialCoeffs, @NotNull PIDCoefficients lateralCoeffs, @NotNull PIDCoefficients headingCoeffs) {
        this(axialCoeffs, lateralCoeffs, headingCoeffs, (Pose2d)null, 0.0D, (NanoClock)null);
    }
}

@Metadata(
        mv = {1, 4, 2},
        bv = {1, 0, 3},
        k = 2,
        d1 = {"\u0000\n\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0004\u001a\u0016\u0010\u0000\u001a\u00020\u00012\u0006\u0010\u0002\u001a\u00020\u00012\u0006\u0010\u0003\u001a\u00020\u0001\u001a\u0016\u0010\u0004\u001a\u00020\u00012\u0006\u0010\u0002\u001a\u00020\u00012\u0006\u0010\u0003\u001a\u00020\u0001¨\u0006\u0005"},
        d2 = {"calculateFieldPoseError", "Lcom/acmerobotics/roadrunner/geometry/Pose2d;", "targetFieldPose", "currentFieldPose", "calculateRobotPoseError", "tnw-UltimateGoal_.TeamCode"}
)
final class HolonomicPIDVAFollower2Kt {
    @NotNull
    public static final Pose2d calculateFieldPoseError(@NotNull Pose2d targetFieldPose, @NotNull Pose2d currentFieldPose) {
        //Intrinsics.checkNotNullParameter(targetFieldPose, "targetFieldPose");
        //Intrinsics.checkNotNullParameter(currentFieldPose, "currentFieldPose");
        return new Pose2d(targetFieldPose.minus(currentFieldPose).vec(), Angle.normDelta(targetFieldPose.getHeading() - currentFieldPose.getHeading()));
    }

    @NotNull
    public static final Pose2d calculateRobotPoseError(@NotNull Pose2d targetFieldPose, @NotNull Pose2d currentFieldPose) {
        //Intrinsics.checkNotNullParameter(targetFieldPose, "targetFieldPose");
        //Intrinsics.checkNotNullParameter(currentFieldPose, "currentFieldPose");
        Pose2d errorInFieldFrame = calculateFieldPoseError(targetFieldPose, currentFieldPose);
        return new Pose2d(errorInFieldFrame.vec().rotated(-currentFieldPose.getHeading()), errorInFieldFrame.getHeading());
    }
}

package frc.robot.subsystems;

/**
 * Shoot-while-moving aim helper for the turreted TestChassis.
 *
 * <p>Stationary hub shots were already solid; strafing left/right misses were usually
 * under-led. This isolates the lead math so flight time, release latency, and lead gain
 * can be tuned without rewriting {@link Turret#calcDist()}.
 *
 * <p>Model: FUEL inherits chassis velocity, so aim as if shooting from the robot's
 * predicted pose at release+flight ({@code pose + v_field * t_lead}) at the real hub.
 * That is equivalent to aiming at the virtual target {@code hub - v_field * t_lead}
 * from the current pose.
 *
 * <p>ChassisSpeeds from CTRE {@code getState().Speeds} are robot-relative; they are
 * rotated into field coordinates with a proper rotation matrix (not atan2+yaw alone
 * with edge-case zeros).
 */
public final class ShootOnMove {
    private ShootOnMove() {}

    /** Measured approximate ball flight time (seconds). */
    public static double kFlightTimeSec = 1.2;

    /**
     * Extra lead time for sensing → wheels → ball-leave delay (seconds).
     * Increases lead without pretending the ball stays in the air longer.
     */
    public static double kReleaseLatencySec = 0.10;

    /**
     * Multiplier on flight time for empirical under/over-lead.
     * Team currently under-leads while strafing — start slightly above 1.0 and trim.
     */
    public static double kLeadGain = 1.15;

    /** How far ahead (seconds) to predict chassis yaw for turret frame. */
    public static double kYawPredictSec = 0.25;

    /** Feedforward scale: (deg/s of aim setpoint) → fraction of turret motor power. */
    public static double kAimRateFeedforward = 0.004;

    /** Last computed d(targetAngle)/dt estimate (deg/s), for turret feedforward. */
    public static double lastAimRateDegPerSec = 0.0;

    public static final class Result {
        /** Field X of the pose to use for aiming (current pose + lead). */
        public final double leadRobotX;
        /** Field Y of the pose to use for aiming (current pose + lead). */
        public final double leadRobotY;
        /** Predicted chassis yaw (radians) after {@link #kYawPredictSec}. */
        public final double predictedYawRad;
        /** Field-relative chassis velocity X (m/s). */
        public final double fieldVelX;
        /** Field-relative chassis velocity Y (m/s). */
        public final double fieldVelY;
        /** Total lead time used (s). */
        public final double leadTimeSec;
        /**
         * Approximate rate of change of the turret's field-relative bearing to the hub
         * (deg/s). Positive means the aim setpoint is increasing in the same sense as
         * {@link Turret#targetAngleDeg}.
         */
        public final double aimRateDegPerSec;

        public Result(
                double leadRobotX,
                double leadRobotY,
                double predictedYawRad,
                double fieldVelX,
                double fieldVelY,
                double leadTimeSec,
                double aimRateDegPerSec) {
            this.leadRobotX = leadRobotX;
            this.leadRobotY = leadRobotY;
            this.predictedYawRad = predictedYawRad;
            this.fieldVelX = fieldVelX;
            this.fieldVelY = fieldVelY;
            this.leadTimeSec = leadTimeSec;
            this.aimRateDegPerSec = aimRateDegPerSec;
        }
    }

    /** Effective lead time: gain * flight + release latency. */
    public static double leadTimeSec() {
        return Math.max(0.0, kFlightTimeSec * kLeadGain + kReleaseLatencySec);
    }

    /**
     * Robot-relative chassis velocity → field-relative.
     *
     * @param robotVx +X forward in robot frame (m/s)
     * @param robotVy +Y left in robot frame (m/s)
     * @param yawRad chassis field yaw (rad)
     */
    public static double[] robotToFieldVelocity(double robotVx, double robotVy, double yawRad) {
        double c = Math.cos(yawRad);
        double s = Math.sin(yawRad);
        // field = R(yaw) * robot
        double fieldVx = robotVx * c - robotVy * s;
        double fieldVy = robotVx * s + robotVy * c;
        return new double[] {fieldVx, fieldVy};
    }

    /**
     * Compute lead-corrected robot pose and aim-rate feedforward for a hub shot.
     *
     * @param robotX field X (m)
     * @param robotY field Y (m)
     * @param yawRad field yaw (rad)
     * @param robotVx robot-relative vx (m/s)
     * @param robotVy robot-relative vy (m/s)
     * @param yawRateDegPerSec chassis yaw rate from gyro (deg/s), CCW+
     * @param hubX hub field X (m)
     * @param hubY hub field Y (m)
     */
    public static Result compute(
            double robotX,
            double robotY,
            double yawRad,
            double robotVx,
            double robotVy,
            double yawRateDegPerSec,
            double hubX,
            double hubY) {
        double leadT = leadTimeSec();
        double[] fieldVel = robotToFieldVelocity(robotVx, robotVy, yawRad);
        double fieldVx = fieldVel[0];
        double fieldVy = fieldVel[1];

        double leadRobotX = robotX + fieldVx * leadT;
        double leadRobotY = robotY + fieldVy * leadT;

        double predictedYawRad =
                yawRad + Math.toRadians(yawRateDegPerSec) * kYawPredictSec;

        // Bearing used by Turret: atan2(turretX - hubX, turretY - hubY), then combined with yaw.
        // Aim rate ≈ d/dt of that bearing while translating (ignore turret offset for FF).
        double dx = leadRobotX - hubX;
        double dy = leadRobotY - hubY;
        double rangeSq = dx * dx + dy * dy;
        double bearingRateRadPerSec = 0.0;
        if (rangeSq > 1e-6) {
            // atan2(dx, dy): d/dt = (dy * dx/dt - dx * dy/dt) / (dx^2 + dy^2)
            // with dx/dt ≈ fieldVx, dy/dt ≈ fieldVy (lead pose tracks chassis velocity)
            bearingRateRadPerSec = (dy * fieldVx - dx * fieldVy) / rangeSq;
        }
        // Turret targetAngleDeg ≈ -deg(atan2(dx,dy) + yaw) (+ wraps/flip). Differentiating:
        // d(target)/dt ≈ -deg(bearingRate) - yawRate
        double aimRateDegPerSec =
                -Math.toDegrees(bearingRateRadPerSec) - yawRateDegPerSec;

        lastAimRateDegPerSec = aimRateDegPerSec;
        return new Result(
                leadRobotX,
                leadRobotY,
                predictedYawRad,
                fieldVx,
                fieldVy,
                leadT,
                aimRateDegPerSec);
    }

    /** Clamp a small feedforward power from the latest aim rate. */
    public static double turretAimFeedforwardPower(double maxPower) {
        double ff = kAimRateFeedforward * lastAimRateDegPerSec;
        if (ff > maxPower) {
            return maxPower;
        }
        if (ff < -maxPower) {
            return -maxPower;
        }
        return ff;
    }
}

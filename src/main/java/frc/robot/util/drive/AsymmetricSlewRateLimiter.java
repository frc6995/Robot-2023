// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drive;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class AsymmetricSlewRateLimiter {
  private final double m_positiveMagnitudeRateLimit;
  private final double m_negativeMagnitudeRateLimit;
  private double m_prevVal;
  private double m_prevTime;

  /**
   * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
   * value.
   *
   * @param positiveMagnitudeRateLimit The rate-of-change limit in the positive direction, in units per
   *     second. This is expected to be positive.
   * @param negativeMagnitudeRateLimit The rate-of-change limit in the negative direction, in units per
   *     second. This is expected to be negative.
   * @param initialValue The initial value of the input.
   */
  public AsymmetricSlewRateLimiter(double positiveMagnitudeRateLimit, double negativeMagnitudeRateLimit, double initialValue) {
    m_positiveMagnitudeRateLimit = positiveMagnitudeRateLimit;
    m_negativeMagnitudeRateLimit = negativeMagnitudeRateLimit;
    m_prevVal = initialValue;
    m_prevTime = MathSharedStore.getTimestamp();
  }
  public AsymmetricSlewRateLimiter(double positiveMagnitudeRateLimit, double negativeMagnitudeRateLimit) {
    m_positiveMagnitudeRateLimit = positiveMagnitudeRateLimit;
    m_negativeMagnitudeRateLimit = negativeMagnitudeRateLimit;
    m_prevVal = 0;
    m_prevTime = MathSharedStore.getTimestamp();
  }

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
   * -rateLimit and initial value.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   * @param initalValue The initial value of the input.
   * @deprecated Use SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double
   *     initalValue) instead.
   */

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
   * -rateLimit.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public AsymmetricSlewRateLimiter(double rateLimit) {
    this(rateLimit, rateLimit, 0);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - m_prevTime;
    double distanceToCover = input - m_prevVal;
    // if decreasing while still positive, or increasing while still negative
    if (Math.signum(m_prevVal) == -Math.signum(distanceToCover)) {
        m_prevVal +=
        MathUtil.clamp(
            input - m_prevVal,
            -Math.abs(m_negativeMagnitudeRateLimit) * elapsedTime,
            Math.abs(m_negativeMagnitudeRateLimit) * elapsedTime);
    }
    else {
        m_prevVal +=
        MathUtil.clamp(
            input - m_prevVal,
            -Math.abs(m_positiveMagnitudeRateLimit) * elapsedTime,
            Math.abs(m_positiveMagnitudeRateLimit) * elapsedTime);
    }

    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = MathSharedStore.getTimestamp();
  }
}


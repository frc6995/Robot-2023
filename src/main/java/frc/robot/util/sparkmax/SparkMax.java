/* Copyright 2023 Pike RoboDevils, FRC Team 1018
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE.md file or
 * at https://opensource.org/licenses/MIT. */

package frc.robot.util.sparkmax;

/*
* Adapted from 3005's 2022 Code
* Original source published at https://github.com/FRC3005/Rapid-React-2022-Public/tree/d499655448ed592c85f9cfbbd78336d8841f46e2
*/

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;


public class SparkMax extends CANSparkMax {

	private List<BiFunction<CANSparkMax, Boolean, Boolean>> m_mutatorChain;
	private List<SparkMax> m_followers = new ArrayList<>();
	private final int kParameterSetAttemptCount = 5;

	/**
	 * Store a reference to every spark max.
	 *
	 * <p>
	 * TODO: Release the reference if it is closed.
	 */
	private static List<SparkMax> m_sparkMaxes = new ArrayList<>();

	private static int m_burnFlashCnt = 0;

	private static boolean allConfigsSuccessful = true;

	/**
	 * Monitor the Spark Max to check for reset. This is used by the health monitor
	 * to automatically
	 * re-initialize the spark max in case of reboot.
	 *
	 * @param sparkMax Spark Max object to monitor
	 * @return True if the device has reset
	 */
	private static boolean sparkmaxMonitorFunction(CANSparkMax sparkMax) {
		return sparkMax.getStickyFault(FaultID.kHasReset);
	}

	/**
	 * Reinitialize the SparkMax by running through all mutations on the object in
	 * order.
	 *
	 * @return true if reinitialized correctly
	 */
	private boolean reinitFunction() {
		for (var fcn : m_mutatorChain) {
			if (!fcn.apply(this, false)) {
				return false;
			}
		}
		return clearFaults() == REVLibError.kOk;
	}

	/**
	 * Create a motor controller from a CANSparkMax object.
	 *
	 * @param sparkMax     A CANSparkMax object. Run any initialization that you
	 *                     want excluded from the
	 *                     built in health monitor functions
	 * @param initFunction A function which takes a CANSparkMax and returns a
	 *                     Boolean. This function
	 *                     is used to initialize the CANSparkMax device, and is
	 *                     called in one of two places. 1) It is
	 *                     called in this constructor, and 2) it is called in the
	 *                     case of a health monitor timeout
	 *                     (i.e. the controller has reset)
	 */
	public SparkMax(int canId, MotorType motorType) {
		super(canId, motorType);

		// Always start fresh and apply settings in code for each device
		// Add delay to avoid any possible timing issues.
		Timer.delay(0.050);

		// If a parameter set fails, this will add more time to alleviate any bus
		// traffic
		// default is 20ms
		setCANTimeout(0);
		m_mutatorChain = new ArrayList<>();
		// withSettings(
		// 	// SparkMaxInitializers.statusFramePeriod(PeriodicFrame.kStatus0, FrameRate.Fast.period),
		// 	// SparkMaxInitializers.statusFramePeriod(PeriodicFrame.kStatus1, FrameRate.Off.period),
		// 	// SparkMaxInitializers.statusFramePeriod(PeriodicFrame.kStatus2, FrameRate.Off.period),
		// 	// SparkMaxInitializers.statusFramePeriod(PeriodicFrame.kStatus3, FrameRate.Off.period),
		// 	// SparkMaxInitializers.statusFramePeriod(PeriodicFrame.kStatus4, FrameRate.Off.period),
		// 	// SparkMaxInitializers.statusFramePeriod(PeriodicFrame.kStatus5, FrameRate.Off.period),
		// 	// SparkMaxInitializers.statusFramePeriod(PeriodicFrame.kStatus6, FrameRate.Off.period)
		// );

		
		m_sparkMaxes.add(this);
	}

	/**
	 * Create a SPARK MAX motor controller with a brushless motor
	 *
	 * @param canId Spark Max CAN Id
	 */
	public SparkMax(int canId) {
		this(canId, MotorType.kBrushless);
	}

	@SafeVarargs
	public final SparkMax withSettings(BiFunction<CANSparkMax, Boolean, Boolean>... initializers) {
		for (BiFunction<CANSparkMax, Boolean, Boolean> initialize : initializers) {
			withInitializer(initialize);
		}
		return this;
	}

	public static BiFunction<CANSparkMax, Boolean, Boolean> check(Function<CANSparkMax, REVLibError> fcn) {
		return (spark, isInit) -> fcn.apply(spark) == REVLibError.kOk;
	}

	public static BiFunction<CANSparkMax, Boolean, Boolean> config(Consumer<CANSparkMax> fcn) {
		return (spark, isInit) -> {
			fcn.accept(spark);
			return true;
		};
	}

	/**
	 * Create spark max object with an initializer method. This method is called on
	 * initialization as
	 * well as if the spark max resets.
	 *
	 * @param initialize function that returns true on success, that takes a
	 *                   CANSparkMax and a Boolean
	 *                   to indicate if the call is from initialization or after a
	 *                   reset.
	 * @return this
	 */
	public SparkMax withInitializer(BiFunction<CANSparkMax, Boolean, Boolean> initialize) {
		m_mutatorChain.add(initialize);

		// Logger.tag("Spark Max").debug("Attempting configuration for SparkMax with ID
		// {}", getDeviceId());
		int setAttemptNumber = 0;
		boolean successful = true;
		while (initialize.apply(this, true) != true) {
			DriverStation.reportWarning(
					String.format("Spark Max ID {}: Failed to initialize, attempt {} of {}",
							getDeviceId(),
							setAttemptNumber,
							kParameterSetAttemptCount),
					false);
			// Logger.tag("Spark Max")
			// .warn(
			// "Spark Max ID {}: Failed to initialize, attempt {} of {}",
			// getDeviceId(),
			// setAttemptNumber,
			// kParameterSetAttemptCount);
			setAttemptNumber++;

			if (setAttemptNumber >= kParameterSetAttemptCount) {
				DriverStation.reportError(String.format("Spark Max ID {}: Failed to initialize!!", getDeviceId()), false);
				// Logger.tag("Spark Max").error("Spark Max ID {}: Failed to initialize!!",
				// getDeviceId());
				successful = false;
				allConfigsSuccessful = false;
				break;
			}
		}
		// if (successful) {
		// 	DriverStation.reportWarning(String.format("Spark Max ID {}: Configuration successful!", getDeviceId()), false);
		// 	if (setAttemptNumber > 0) {
		// 		DriverStation.reportWarning(
		// 				String.format("Spark Max ID {}: Configuration took {} tries.", getDeviceId(), setAttemptNumber + 1), false);
		// 	}
		// }
		return this;
	}

	/**
	 * Modify CANSparkMax object. Mutations using this method are re-run in order in
	 * the case of a
	 * device failure that is later recovered. The same function can be called
	 * multiple times, and
	 * will simply be moved to the end of the list each call.
	 *
	 * <p>
	 * Only adds the function to the list if it succeeds
	 *
	 * @param fcn a function on the underlying CANSparkMax object returning true on
	 *            success. Typically
	 *            used to change parameter values. Function should run quickly and
	 *            return.
	 * @return result of mutate function
	 */
	public boolean mutate(BiFunction<CANSparkMax, Boolean, Boolean> fcn) {
		Boolean result = fcn.apply(this, true);

		int setAttemptNumber = 0;
		while (result == null || result != true) {

			DriverStation.reportWarning(
					String.format("Spark Max ID {}: Failed to run mutator, attempt {} of {}",
							getDeviceId(),
							setAttemptNumber,
							kParameterSetAttemptCount),
					false);
			setAttemptNumber++;

			if (setAttemptNumber >= kParameterSetAttemptCount) {
				DriverStation.reportError(
						String.format("Spark Max ID {}: Failed to run mutator function!!",
								getDeviceId()),
						false);
				// Logger.tag("Spark Max")
				// .error("Spark Max ID {}: Failed to run mutator function!!", getDeviceId());
				allConfigsSuccessful = false;
				break;
			}
			result = fcn.apply(this, true);
		}

		if (result != null && result) {
			m_mutatorChain.add(fcn);
		}
		return result == null ? false : result;
	}

	public enum FrameStrategy {
		kDefault,
		kNoExternalFeedback,
		kVelocity,
		kPosition,
		kNoFeedback,
		kCurrent,
		kVoltage,
		kVelocityAndPosition;
		}


	private enum FrameRate {
		Fast(15),
		Slow(500),
		Off(65535);
		public final int period;
		private FrameRate(int period) {
			this.period = period;
		}
	}

	

	/**
	 * Set a frame strategy for the feedback frames. This changes the periodic frame
	 * rates to be
	 * either slow (500ms) or fast (15ms) depending on the signal needed.
	 *
	 * @param sparkMax     CANSparkMax object to run this on
	 * @param strategy     Signal strategy
	 * @param withFollower does this device have a follower motor
	 */
	public static void setFrameStrategy(
			CANSparkMax sparkMax, FrameStrategy strategy, boolean withFollower) {
		final int slowFrame = 500;
		final int fastFrame = 15;
		int status0 = 10;
		int status1 = 20;
		int status2 = 20;
		int status3 = 50;
		switch (strategy) {
			case kVelocity:
			case kCurrent:
			case kVoltage:
				status1 = fastFrame;
				status2 = slowFrame;
				status3 = slowFrame;
				break;
			case kPosition:
				status1 = slowFrame;
				status2 = fastFrame;
				status3 = slowFrame;
				break;
			case kNoFeedback:
				status1 = slowFrame;
				status2 = slowFrame;
				status3 = slowFrame;
				break;
			case kVelocityAndPosition:
				status1 = fastFrame;
				status2 = fastFrame;
				status3 = slowFrame;
				break;
			case kDefault:
			default:
				// already set
				break;
		}
		if (!withFollower) {
			status0 = slowFrame;
		}
		// sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
		// sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
		// sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
		// sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
	}

	/**
	 * Set a frame strategy for the feedback frames. This changes the periodic frame
	 * rates to be
	 * either slow (500ms) or fast (15ms) depending on the signal needed.
	 *
	 * @param sparkMax CANSparkMax object to run this on
	 * @param strategy Signal strategy
	 */
	public static void setFrameStrategy(CANSparkMax sparkMax, FrameStrategy strategy) {
		setFrameStrategy(sparkMax, strategy, false);
	}

	public SparkMax withFollower(SparkMax follower) {
		return withFollower(follower, false);
	}

	public SparkMax withFollower(SparkMax follower, boolean invert) {

		follower.mutate(check((spark)->spark.follow(this, invert)));
		m_followers.add(follower);
		return this;
	}

	/**
	 * Run burnFlash() for all controllers initialized. The ideal use case for this
	 * call is to call it
	 * once everything has been initialized. The burnFlash() call has the side
	 * effect of preventing
	 * all communication *to* the device for up to 200ms or more, potentially
	 * including some messages
	 * called before the burnFlash() call, and receiveing messages *from* the
	 * device.
	 *
	 * <p>
	 * WARNING: This call will sleep the thread before and after burning flash. This
	 * is for your
	 * safety.
	 */
	public static void burnFlashInSync() {
		// DriverStation.reportWarning(
		// 		String.format("Burning Flash Count: {}", ++m_burnFlashCnt), false);
		// Logger.tag("SparkMax").debug("Burning Flash Count: {}", ++m_burnFlashCnt);
		Timer.delay(0.25);
		for (SparkMax max : m_sparkMaxes) {
			// DriverStation.reportWarning(
			// 		String.format("Burning Flash Count for Can ID {}", max.getDeviceId()), false);
			// Logger.tag("SparkMax").trace("Burning flash for Can ID {}",
			// max.getDeviceId());
			max.burnFlash();
			// Enough time to not spam the bus too bad
			Timer.delay(0.005);
		}
		Timer.delay(0.25);
		DriverStation.reportWarning("Burn Flash Complete", false);
		// Logger.tag("SparkMax").debug("Burn Flash Complete.");
	}

	public static boolean configSuccessful() {
		return allConfigsSuccessful;
	}

	/* Common initializers */
	public static class SparkMaxInitializers {
		public static BiFunction<CANSparkMax, Boolean, Boolean> coast() {
			return check((spark)->spark.setIdleMode(CANSparkMax.IdleMode.kCoast));
		}
		public static BiFunction<CANSparkMax, Boolean, Boolean> brake() {
			return check((spark)->spark.setIdleMode(CANSparkMax.IdleMode.kBrake));}
		
		public static BiFunction<CANSparkMax, Boolean, Boolean> currentLimit(int currentLimit){return check((spark)->spark.setSmartCurrentLimit(currentLimit));}
		
		public static BiFunction<CANSparkMax, Boolean, Boolean> invert(){return config((spark)->spark.setInverted(true));}

		/**  */
		public static BiFunction<CANSparkMax, Boolean, Boolean> 
			statusFramePeriod(PeriodicFrame frame, int period){
				return config((spark)->spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, period));}

		public static BiFunction<CANSparkMax, Boolean, Boolean> 
		softLimit(SoftLimitDirection direction, double value){
			return check((spark)->{
				spark.enableSoftLimit(direction, true);
				return spark.setSoftLimit(direction, (float) value);}
			);}
		public static BiFunction<CANSparkMax, Boolean, Boolean> 
		absEncoderPositionConversion(double value){
			return check((spark)->
				spark.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(value)
			);}
}
	}
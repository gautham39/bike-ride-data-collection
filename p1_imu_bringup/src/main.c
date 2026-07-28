/*
 * p1_imu_bringup - IMU data collection on the Seeed XIAO nRF54L15
 *
 * Reads the on-board LSM6DS3TR-C 6-axis IMU (i2c30 @ 0x6a, devicetree alias
 * "imu0", compatible "st,lsm6dsl") and streams accelerometer + gyroscope
 * samples to the console UART as CSV.
 *
 * The gyro zero-rate bias is measured at startup and subtracted from every
 * logged sample, so the board must be held still for the first few seconds
 * after power-on. See gyro_calibrate().
 *
 * Board:   xiao_nrf54l15/nrf54l15/cpuapp
 * Console: uart20 @ 115200 (P1.09 TX / P1.08 RX -> on-board USB serial)
 *
 * Build:   west build -b xiao_nrf54l15/nrf54l15/cpuapp -p
 * Flash:   west flash
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(imu_bringup, LOG_LEVEL_INF);

/*
 * Hardware output data rate. The LSM6DSL driver powers the sensor down at init
 * when CONFIG_LSM6DSL_ACCEL_ODR / _GYRO_ODR are 0 (the default), so the ODR
 * must be set at runtime or sample_fetch() returns stale zeroes forever.
 *
 * Valid ODRs: 12, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660 Hz.
 */
#define IMU_ODR_HZ 104

/*
 * Sample/print cadence. Kept at half the ODR so every read returns a fresh
 * sample (no duplicates) and so the CSV stream stays well inside the 115200
 * baud console budget: ~75 bytes/line * 50 Hz = ~3.8 kB/s of ~11.5 kB/s.
 */
#define SAMPLE_RATE_HZ 50
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)

/* Full-scale ranges. +/-4 g covers road vibration and pothole impacts without
 * clipping; +/-500 dps covers lean and steering rates with headroom.
 */
#define ACCEL_RANGE_G 4
#define GYRO_RANGE_DPS 500

/* Log a one-line status summary every this many samples. */
#define STATUS_EVERY_N_SAMPLES (SAMPLE_RATE_HZ * 10)

/*
 * Gyro zero-rate bias calibration.
 *
 * The LSM6DSL driver applies a sensitivity scale and nothing else - it has no
 * offset support at all - so a stationary gyro reads a small nonzero rate that
 * varies part to part and with temperature. Averaging N stationary samples
 * estimates that offset; subtracting it centres the logged rate on zero.
 *
 * Noise in the estimate falls as 1/sqrt(N), so more samples buys a better bias
 * at the cost of a longer hold. 200 samples is ~4 s at SAMPLE_RATE_HZ.
 */
#define GYRO_CAL_SAMPLES 200

/*
 * Motion guard. If any axis swings more than this peak-to-peak while
 * calibrating, the board was not still and the estimate has real rotation
 * averaged into it. 0.02 rad/s is ~1.1 deg/s: well above the sensor's
 * stationary noise, well below anything you'd cause by hand.
 */
#define GYRO_CAL_MOTION_LIMIT 0.02

/* Abandon calibration if this many reads fail while collecting. */
#define GYRO_CAL_MAX_ERRORS 10

struct imu_sample {
	uint32_t timestamp_ms;
	double accel[3]; /* m/s^2 */
	double gyro[3];  /* rad/s */
#if IS_ENABLED(CONFIG_LSM6DSL_ENABLE_TEMP)
	double die_temp; /* degrees C */
#endif
};

static int imu_configure(const struct device *imu)
{
	struct sensor_value val;
	int err;

	sensor_g_to_ms2(ACCEL_RANGE_G, &val);
	err = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &val);
	if (err) {
		LOG_ERR("Failed to set accel full-scale to +/-%d g (err %d)", ACCEL_RANGE_G, err);
		return err;
	}

	sensor_degrees_to_rad(GYRO_RANGE_DPS, &val);
	err = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &val);
	if (err) {
		LOG_ERR("Failed to set gyro full-scale to +/-%d dps (err %d)", GYRO_RANGE_DPS,
			err);
		return err;
	}

	val.val1 = IMU_ODR_HZ;
	val.val2 = 0;
	err = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	if (err) {
		LOG_ERR("Failed to set accel ODR to %d Hz (err %d)", IMU_ODR_HZ, err);
		return err;
	}

	err = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	if (err) {
		LOG_ERR("Failed to set gyro ODR to %d Hz (err %d)", IMU_ODR_HZ, err);
		return err;
	}

	return 0;
}

static int imu_read(const struct device *imu, struct imu_sample *sample)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int err;

	err = sensor_sample_fetch(imu);
	if (err) {
		return err;
	}

	err = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (err) {
		return err;
	}

	err = sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (err) {
		return err;
	}

	sample->timestamp_ms = k_uptime_get_32();

	for (int i = 0; i < 3; i++) {
		sample->accel[i] = sensor_value_to_double(&accel[i]);
		sample->gyro[i] = sensor_value_to_double(&gyro[i]);
	}

#if IS_ENABLED(CONFIG_LSM6DSL_ENABLE_TEMP)
	struct sensor_value temp;

	err = sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP, &temp);
	if (err) {
		return err;
	}
	sample->die_temp = sensor_value_to_double(&temp);
#endif

	return 0;
}

/*
 * Estimate the gyro zero-rate bias by averaging stationary samples. Tracks the
 * peak-to-peak spread as it goes so it can tell you when the result is not
 * trustworthy. Returns 0 and fills bias[] even in that case - a warned-about
 * bias is more useful than none, and you can discard the session yourself.
 */
static int gyro_calibrate(const struct device *imu, double bias[3])
{
	double sum[3] = {0.0, 0.0, 0.0};
	double min[3] = {0.0, 0.0, 0.0};
	double max[3] = {0.0, 0.0, 0.0};
	double peak = 0.0;
	uint32_t errors = 0;
	int collected = 0;

	LOG_INF("Calibrating gyro - hold still for ~%d ms...",
		GYRO_CAL_SAMPLES * SAMPLE_PERIOD_MS);

	while (collected < GYRO_CAL_SAMPLES) {
		struct imu_sample sample;
		int err = imu_read(imu, &sample);

		if (err) {
			if (++errors > GYRO_CAL_MAX_ERRORS) {
				LOG_ERR("Too many read errors while calibrating (last err %d)",
					err);
				return err;
			}
			k_msleep(SAMPLE_PERIOD_MS);
			continue;
		}

		for (int i = 0; i < 3; i++) {
			sum[i] += sample.gyro[i];

			if (collected == 0 || sample.gyro[i] < min[i]) {
				min[i] = sample.gyro[i];
			}
			if (collected == 0 || sample.gyro[i] > max[i]) {
				max[i] = sample.gyro[i];
			}
		}

		collected++;
		k_msleep(SAMPLE_PERIOD_MS);
	}

	for (int i = 0; i < 3; i++) {
		bias[i] = sum[i] / (double)GYRO_CAL_SAMPLES;

		double range = max[i] - min[i];

		if (range > peak) {
			peak = range;
		}
	}

	LOG_INF("Gyro bias: %.4f %.4f %.4f rad/s", bias[0], bias[1], bias[2]);

	if (peak > GYRO_CAL_MOTION_LIMIT) {
		LOG_WRN("Moved while calibrating (peak %.4f > %.4f rad/s) - bias is "
			"contaminated; power-cycle while still to redo",
			peak, (double)GYRO_CAL_MOTION_LIMIT);
	} else {
		LOG_INF("Calibration OK (peak deviation %.4f rad/s)", peak);
	}

	return 0;
}

static void print_csv_header(void)
{
	printk("# XIAO nRF54L15 IMU stream: accel m/s^2 (+/-%d g), gyro rad/s (+/-%d dps, "
	       "bias-corrected), ODR %d Hz, sampled %d Hz\n",
	       ACCEL_RANGE_G, GYRO_RANGE_DPS, IMU_ODR_HZ, SAMPLE_RATE_HZ);
#if IS_ENABLED(CONFIG_LSM6DSL_ENABLE_TEMP)
	printk("timestamp_ms,ax,ay,az,gx,gy,gz,die_temp_c\n");
#else
	printk("timestamp_ms,ax,ay,az,gx,gy,gz\n");
#endif
}

static void print_csv_sample(const struct imu_sample *s)
{
	printk("%u,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f"
#if IS_ENABLED(CONFIG_LSM6DSL_ENABLE_TEMP)
	       ",%.2f"
#endif
	       "\n",
	       s->timestamp_ms, s->accel[0], s->accel[1], s->accel[2], s->gyro[0], s->gyro[1],
	       s->gyro[2]
#if IS_ENABLED(CONFIG_LSM6DSL_ENABLE_TEMP)
	       ,
	       s->die_temp
#endif
	);
}

int main(void)
{
	const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));
	struct imu_sample sample;
	double gyro_bias[3];
	uint32_t sample_count = 0;
	uint32_t error_count = 0;
	int64_t next_deadline;

	LOG_INF("XIAO nRF54L15 IMU bringup starting");

	if (!device_is_ready(imu)) {
		LOG_ERR("IMU %s is not ready - check i2c30 and the pdm_imu_pwr rail (P0.01)",
			imu->name);
		return -ENODEV;
	}

	LOG_INF("IMU %s ready", imu->name);

	if (imu_configure(imu)) {
		return -EIO;
	}

	LOG_INF("Configured: ODR %d Hz, accel +/-%d g, gyro +/-%d dps", IMU_ODR_HZ, ACCEL_RANGE_G,
		GYRO_RANGE_DPS);

	/* Let the first samples settle after the ODR change before calibrating -
	 * the very first reads after a power-down-to-active transition are not
	 * representative, and averaging them in would skew the bias.
	 */
	k_msleep(100);

	if (gyro_calibrate(imu, gyro_bias)) {
		return -EIO;
	}

	print_csv_header();

	/* Absolute deadlines rather than a fixed k_msleep so print time does not
	 * accumulate as drift in the sample interval.
	 */
	next_deadline = k_uptime_get();

	while (1) {
		next_deadline += SAMPLE_PERIOD_MS;

		int err = imu_read(imu, &sample);

		if (err) {
			error_count++;
			LOG_WRN("IMU read failed (err %d, %u total)", err, error_count);
		} else {
			for (int i = 0; i < 3; i++) {
				sample.gyro[i] -= gyro_bias[i];
			}

			print_csv_sample(&sample);

			if (++sample_count % STATUS_EVERY_N_SAMPLES == 0) {
				LOG_INF("%u samples streamed, %u read errors", sample_count,
					error_count);
			}
		}

		int32_t remaining = (int32_t)(next_deadline - k_uptime_get());

		if (remaining > 0) {
			k_msleep(remaining);
		} else {
			/* Fell behind (console backpressure); resync instead of
			 * spinning to catch up.
			 */
			next_deadline = k_uptime_get();
		}
	}

	return 0;
}

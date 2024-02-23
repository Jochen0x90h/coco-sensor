#pragma once

#include "Sensor.hpp"
#include <coco/Loop.hpp>
#include <coco/Buffer.hpp>
#include <cstdint>


namespace coco {

/**
 * Controller for BME680 air sensor connected via I2C or SPI
 * Measures temperature, pressure, humidity and air quality
 * https://github.com/BoschSensortec/BME68x-Sensor-API
 * deprecated: https://github.com/BoschSensortec/BME680_driver
 */
class BME680 : public Sensor {
public:
	// minimum size of buffer
	static constexpr int BUFFER_SIZE = 24;

	enum class Mode : uint8_t {
		I2C,
		SPI
	};

	// oversampling
	enum class Oversampling : uint8_t {
		// measurement is disabled
		OFF = 0,
		X1 = 1,
		X2 = 2,
		X4 = 3,
		X8 = 4,
		X16 = 5
	};

	// filter coefficient
	enum class Filter : uint8_t {
		C0 = 0,
		C1 = 1,
		C3 = 2,
		C7 = 3,
		C15 = 4,
		C31 = 5,
		C63 = 6,
		C127 = 7
	};

	struct Config {
		Mode mode;

		// temperature oversampling
		Oversampling temperatureOversampling;

		// pressure oversampling
		Oversampling pressureOversampling;

		// iir filter for temperature and pressure (0-7)
		Filter filter;

		// humidity oversampling
		Oversampling humidityOversampling;

		// heater temperature for gas sensor, 200 to 400 degrees
		int heaterTemperature;

		// time between the beginning of the heat phase and the start of gas sensor resistance measurement
		Milliseconds<> heaterDuration;
	};

	/**
	 * Constructor
	 * @param loop event loop
	 * @param buffer transfer buffer of an SPI or I2C device of at least BUFFER_SIZE
	 * @param config config
	 */
	BME680(Loop &loop, Buffer &buffer, const Config &config);
	~BME680() override;

	float getTemperature() {return this->values[0];}
	float getHumidity() {return this->values[1];}
	float getPressure() {return this->values[2];}
	float getGasResistance() {return this->values[3];}

	int get(const Array<float> &values) override;

	/**
	 * Wait until measurement values have changed
	 * @return use co_await on return value to wait for a value change
	 */
	[[nodiscard]] Awaitable<> changed() override;

protected:
	static void get_calib_data(BME680 *dev, uint8_t *coeff_array);
	static uint8_t calc_res_heat(const BME680 *dev, int temp);
	static uint8_t calc_gas_wait(int dur);
	static float calc_temperature(BME680 *dev, uint32_t temp_adc);
	static float calc_pressure(BME680 *dev, uint32_t pres_adc);
	static float calc_humidity(BME680 *dev, uint16_t hum_adc);
	static float calc_gas_resistance_low(BME680 *dev, uint16_t gas_res_adc, uint8_t gas_range);

	Coroutine measure(int spi);

	Loop &loop;
	Buffer &buffer;

	uint16_t heaterTemperature;

	// precomputed config registers
	uint8_t regConfig;
	uint8_t regCtrlMeas;
	uint8_t regCtrlHum;
	uint8_t regGasWait;

	/*
	* @brief Structure to hold the calibration coefficients
	*/
	struct bme68x_calib_data
	{
		/*! Calibration coefficient for the humidity sensor */
		uint16_t par_h1;

		/*! Calibration coefficient for the humidity sensor */
		uint16_t par_h2;

		/*! Calibration coefficient for the humidity sensor */
		int8_t par_h3;

		/*! Calibration coefficient for the humidity sensor */
		int8_t par_h4;

		/*! Calibration coefficient for the humidity sensor */
		int8_t par_h5;

		/*! Calibration coefficient for the humidity sensor */
		uint8_t par_h6;

		/*! Calibration coefficient for the humidity sensor */
		int8_t par_h7;

		/*! Calibration coefficient for the gas sensor */
		int8_t par_gh1;

		/*! Calibration coefficient for the gas sensor */
		int16_t par_gh2;

		/*! Calibration coefficient for the gas sensor */
		int8_t par_gh3;

		/*! Calibration coefficient for the temperature sensor */
		uint16_t par_t1;

		/*! Calibration coefficient for the temperature sensor */
		int16_t par_t2;

		/*! Calibration coefficient for the temperature sensor */
		int8_t par_t3;

		/*! Calibration coefficient for the pressure sensor */
		uint16_t par_p1;

		/*! Calibration coefficient for the pressure sensor */
		int16_t par_p2;

		/*! Calibration coefficient for the pressure sensor */
		int8_t par_p3;

		/*! Calibration coefficient for the pressure sensor */
		int16_t par_p4;

		/*! Calibration coefficient for the pressure sensor */
		int16_t par_p5;

		/*! Calibration coefficient for the pressure sensor */
		int8_t par_p6;

		/*! Calibration coefficient for the pressure sensor */
		int8_t par_p7;

		/*! Calibration coefficient for the pressure sensor */
		int16_t par_p8;

		/*! Calibration coefficient for the pressure sensor */
		int16_t par_p9;

		/*! Calibration coefficient for the pressure sensor */
		uint8_t par_p10;

		/*! Variable to store the intermediate temperature coefficient */
		float t_fine;

		/*! Heater resistance range coefficient */
		uint8_t res_heat_range;

		/*! Heater resistance value coefficient */
		int8_t res_heat_val;

		/*! Gas resistance range switching error coefficient */
		int8_t range_sw_err;
	};

	bme68x_calib_data calib;

	// Ambient temperature in Degree C
	int8_t amb_temp = 25;

	// measurement values
	int frameNumber = 0;
	float values[4];

	// coroutines to resume on state change
	CoroutineTaskList<> changeTasks;
};

} // namespace coco

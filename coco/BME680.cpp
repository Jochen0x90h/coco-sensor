#include "BME680.hpp"
#include <coco/debug.hpp>


namespace coco {

#define READ(reg) (reg | spi)
#define WRITE(reg) (reg & ~spi)


// page select
constexpr uint8_t BME68X_REG_MEM_PAGE = 0x73;


// page 0
// ------

// chip id
constexpr uint8_t BME68X_REG_CHIP_ID = 0xD0;
constexpr uint8_t BME68X_CHIP_ID = 0x61;

// soft reset
constexpr uint8_t BME68X_REG_SOFT_RESET = 0xE0;
constexpr uint8_t BME68X_SOFT_RESET_CMD = 0xB6;

// 1st group of coefficients
constexpr uint8_t BME68X_REG_COEFF1 = 0x8A;
constexpr uint8_t BME68X_LEN_COEFF1 = 23;

constexpr uint8_t BME68X_REG_COEFF2 = 0xE1;
constexpr uint8_t BME68X_LEN_COEFF2 = 14;


// page 1
// ------

constexpr uint8_t BME68X_REG_COEFF3 = 0x00;
constexpr uint8_t BME68X_LEN_COEFF3 = 5;

// heater resistance and wait addresses
constexpr uint8_t BME68X_REG_RES_HEAT0 = 0x5A;
constexpr uint8_t BME68X_REG_GAS_WAIT0 = 0x64;

constexpr uint8_t BME68X_REG_CTRL_GAS_0 = 0x70;
constexpr uint8_t BME68X_REG_CTRL_GAS_1 = 0x71;
constexpr uint8_t BME68X_REG_CTRL_HUM = 0x72;
constexpr uint8_t BME68X_REG_CTRL_MEAS = 0x74;
constexpr uint8_t BME68X_REG_CONFIG = 0x75;


constexpr uint8_t BME68X_LEN_COEFF_ALL = 42;


/* Macro to combine two 8 bit data's to form a 16 bit data */
#define BME68X_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)


/* Coefficient index macros */

/* Coefficient T2 LSB position */
#define BME68X_IDX_T2_LSB                         (0)

/* Coefficient T2 MSB position */
#define BME68X_IDX_T2_MSB                         (1)

/* Coefficient T3 position */
#define BME68X_IDX_T3                             (2)

/* Coefficient P1 LSB position */
#define BME68X_IDX_P1_LSB                         (4)

/* Coefficient P1 MSB position */
#define BME68X_IDX_P1_MSB                         (5)

/* Coefficient P2 LSB position */
#define BME68X_IDX_P2_LSB                         (6)

/* Coefficient P2 MSB position */
#define BME68X_IDX_P2_MSB                         (7)

/* Coefficient P3 position */
#define BME68X_IDX_P3                             (8)

/* Coefficient P4 LSB position */
#define BME68X_IDX_P4_LSB                         (10)

/* Coefficient P4 MSB position */
#define BME68X_IDX_P4_MSB                         (11)

/* Coefficient P5 LSB position */
#define BME68X_IDX_P5_LSB                         (12)

/* Coefficient P5 MSB position */
#define BME68X_IDX_P5_MSB                         (13)

/* Coefficient P7 position */
#define BME68X_IDX_P7                             (14)

/* Coefficient P6 position */
#define BME68X_IDX_P6                             (15)

/* Coefficient P8 LSB position */
#define BME68X_IDX_P8_LSB                         (18)

/* Coefficient P8 MSB position */
#define BME68X_IDX_P8_MSB                         (19)

/* Coefficient P9 LSB position */
#define BME68X_IDX_P9_LSB                         (20)

/* Coefficient P9 MSB position */
#define BME68X_IDX_P9_MSB                         (21)

/* Coefficient P10 position */
#define BME68X_IDX_P10                            (22)

/* Coefficient H2 MSB position */
#define BME68X_IDX_H2_MSB                         (23)

/* Coefficient H2 LSB position */
#define BME68X_IDX_H2_LSB                         (24)

/* Coefficient H1 LSB position */
#define BME68X_IDX_H1_LSB                         (24)

/* Coefficient H1 MSB position */
#define BME68X_IDX_H1_MSB                         (25)

/* Coefficient H3 position */
#define BME68X_IDX_H3                             (26)

/* Coefficient H4 position */
#define BME68X_IDX_H4                             (27)

/* Coefficient H5 position */
#define BME68X_IDX_H5                             (28)

/* Coefficient H6 position */
#define BME68X_IDX_H6                             (29)

/* Coefficient H7 position */
#define BME68X_IDX_H7                             (30)

/* Coefficient T1 LSB position */
#define BME68X_IDX_T1_LSB                         (31)

/* Coefficient T1 MSB position */
#define BME68X_IDX_T1_MSB                         (32)

/* Coefficient GH2 LSB position */
#define BME68X_IDX_GH2_LSB                        (33)

/* Coefficient GH2 MSB position */
#define BME68X_IDX_GH2_MSB                        (34)

/* Coefficient GH1 position */
#define BME68X_IDX_GH1                            (35)

/* Coefficient GH3 position */
#define BME68X_IDX_GH3                            (36)

/* Coefficient res heat value position */
#define BME68X_IDX_RES_HEAT_VAL                   (37)

/* Coefficient res heat range position */
#define BME68X_IDX_RES_HEAT_RANGE                 (39)

/* Coefficient range switching error position */
#define BME68X_IDX_RANGE_SW_ERR                   (41)


/* Mask for the H1 calibration coefficient */
#define BME68X_BIT_H1_DATA_MSK                    UINT8_C(0x0f)

/* Mask for res heat range */
#define BME68X_RHRANGE_MSK                        UINT8_C(0x30)

/* Mask for range switching error */
#define BME68X_RSERROR_MSK                        UINT8_C(0xf0)



BME680::BME680(Loop &loop, Buffer &buffer, const Config &config)
	: loop(loop)
	, buffer(buffer)
	, heaterTemperature(config.heaterTemperature)
	, values{}
{
	this->regConfig = int(config.filter) << 2; // 3-wire SPI (spi_3w_en) stays off
	this->regCtrlMeas = (int(config.temperatureOversampling) << 5) | (int(config.pressureOversampling) << 2);
	this->regCtrlHum = int(config.humidityOversampling);
	this->regGasWait = calc_gas_wait(config.heaterDuration.value);

	// start measure coroutine
	measure((config.mode == Mode::SPI) ? 0x80 : 0);
}

BME680::~BME680() {
	// todo: stop coroutine
}

int BME680::get(const Array<float> &values) {
	int count = std::min(values.size(), 4);
	for (int i = 0; i < count; ++i) {
		values[i] = this->values[i];
	}
	return this->frameNumber;
}

Awaitable<> BME680::changed() {
	return {this->changeTasks};
}

void BME680::get_calib_data(BME680 *dev, uint8_t *coeff_array) {
	/* Temperature related coefficients */
	dev->calib.par_t1 =
		(uint16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T1_MSB], coeff_array[BME68X_IDX_T1_LSB]));
	dev->calib.par_t2 =
		(int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T2_MSB], coeff_array[BME68X_IDX_T2_LSB]));
	dev->calib.par_t3 = (int8_t)(coeff_array[BME68X_IDX_T3]);

	/* Pressure related coefficients */
	dev->calib.par_p1 =
		(uint16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P1_MSB], coeff_array[BME68X_IDX_P1_LSB]));
	dev->calib.par_p2 =
		(int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P2_MSB], coeff_array[BME68X_IDX_P2_LSB]));
	dev->calib.par_p3 = (int8_t)coeff_array[BME68X_IDX_P3];
	dev->calib.par_p4 =
		(int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P4_MSB], coeff_array[BME68X_IDX_P4_LSB]));
	dev->calib.par_p5 =
		(int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P5_MSB], coeff_array[BME68X_IDX_P5_LSB]));
	dev->calib.par_p6 = (int8_t)(coeff_array[BME68X_IDX_P6]);
	dev->calib.par_p7 = (int8_t)(coeff_array[BME68X_IDX_P7]);
	dev->calib.par_p8 =
		(int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P8_MSB], coeff_array[BME68X_IDX_P8_LSB]));
	dev->calib.par_p9 =
		(int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P9_MSB], coeff_array[BME68X_IDX_P9_LSB]));
	dev->calib.par_p10 = (uint8_t)(coeff_array[BME68X_IDX_P10]);

	/* Humidity related coefficients */
	dev->calib.par_h1 =
		(uint16_t)(((uint16_t)coeff_array[BME68X_IDX_H1_MSB] << 4) |
					(coeff_array[BME68X_IDX_H1_LSB] & BME68X_BIT_H1_DATA_MSK));
	dev->calib.par_h2 =
		(uint16_t)(((uint16_t)coeff_array[BME68X_IDX_H2_MSB] << 4) | ((coeff_array[BME68X_IDX_H2_LSB]) >> 4));
	dev->calib.par_h3 = (int8_t)coeff_array[BME68X_IDX_H3];
	dev->calib.par_h4 = (int8_t)coeff_array[BME68X_IDX_H4];
	dev->calib.par_h5 = (int8_t)coeff_array[BME68X_IDX_H5];
	dev->calib.par_h6 = (uint8_t)coeff_array[BME68X_IDX_H6];
	dev->calib.par_h7 = (int8_t)coeff_array[BME68X_IDX_H7];

	/* Gas heater related coefficients */
	dev->calib.par_gh1 = (int8_t)coeff_array[BME68X_IDX_GH1];
	dev->calib.par_gh2 =
		(int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_GH2_MSB], coeff_array[BME68X_IDX_GH2_LSB]));
	dev->calib.par_gh3 = (int8_t)coeff_array[BME68X_IDX_GH3];

	/* Other coefficients */
	dev->calib.res_heat_range = ((coeff_array[BME68X_IDX_RES_HEAT_RANGE] & BME68X_RHRANGE_MSK) / 16);
	dev->calib.res_heat_val = (int8_t)coeff_array[BME68X_IDX_RES_HEAT_VAL];
	dev->calib.range_sw_err = ((int8_t)(coeff_array[BME68X_IDX_RANGE_SW_ERR] & BME68X_RSERROR_MSK)) / 16;
}

uint8_t BME680::calc_res_heat(const BME680 *dev, int temp) {
    uint8_t heatr_res;
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;

    if (temp > 400) /* Cap temperature */
    {
        temp = 400;
    }

    var1 = (((int32_t)dev->amb_temp * dev->calib.par_gh3) / 1000) * 256;
    var2 = (dev->calib.par_gh1 + 784) * (((((dev->calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (dev->calib.res_heat_range + 4));
    var5 = (131 * dev->calib.res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

// calculate gas wait register value from given duration in milliseconds
uint8_t BME680::calc_gas_wait(int dur) {
    uint8_t factor = 0;
    uint8_t durval;

    if (dur >= 0xfc0)
    {
        durval = 0xff; /* Max duration*/
    }
    else
    {
        while (dur > 0x3F)
        {
            dur = dur / 4;
            factor += 1;
        }

        durval = (uint8_t)(dur + (factor * 64));
    }

    return durval;
}

float BME680::calc_temperature(BME680 *dev, uint32_t temp_adc) {
    float var1;
    float var2;
    float calc_temp;

    /* calculate var1 data */
    var1 = ((((float)temp_adc / 16384.0f) - ((float)dev->calib.par_t1 / 1024.0f)) * ((float)dev->calib.par_t2));

    /* calculate var2 data */
    var2 =
        (((((float)temp_adc / 131072.0f) - ((float)dev->calib.par_t1 / 8192.0f)) *
          (((float)temp_adc / 131072.0f) - ((float)dev->calib.par_t1 / 8192.0f))) * ((float)dev->calib.par_t3 * 16.0f));

    /* t_fine value*/
    dev->calib.t_fine = (var1 + var2);

    /* compensated temperature data*/
    calc_temp = ((dev->calib.t_fine) / 5120.0f);

    return calc_temp;
}

float BME680::calc_pressure(BME680 *dev, uint32_t pres_adc) {
    float var1;
    float var2;
    float var3;
    float calc_pres;

    var1 = (((float)dev->calib.t_fine / 2.0f) - 64000.0f);
    var2 = var1 * var1 * (((float)dev->calib.par_p6) / (131072.0f));
    var2 = var2 + (var1 * ((float)dev->calib.par_p5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)dev->calib.par_p4) * 65536.0f);
    var1 = (((((float)dev->calib.par_p3 * var1 * var1) / 16384.0f) + ((float)dev->calib.par_p2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)dev->calib.par_p1));
    calc_pres = (1048576.0f - ((float)pres_adc));

    /* Avoid exception caused by division by zero */
    if ((int)var1 != 0)
    {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
        var1 = (((float)dev->calib.par_p9) * calc_pres * calc_pres) / 2147483648.0f;
        var2 = calc_pres * (((float)dev->calib.par_p8) / 32768.0f);
        var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) * (dev->calib.par_p10 / 131072.0f));
        calc_pres = (calc_pres + (var1 + var2 + var3 + ((float)dev->calib.par_p7 * 128.0f)) / 16.0f);
    }
    else
    {
        calc_pres = 0;
    }

    return calc_pres;
}

float BME680::calc_humidity(BME680 *dev, uint16_t hum_adc) {
    float calc_hum;
    float var1;
    float var2;
    float var3;
    float var4;
    float temp_comp;

    /* compensated temperature data*/
    temp_comp = ((dev->calib.t_fine) / 5120.0f);
    var1 = (float)((float)hum_adc) -
           (((float)dev->calib.par_h1 * 16.0f) + (((float)dev->calib.par_h3 / 2.0f) * temp_comp));
    var2 = var1 *
           ((float)(((float)dev->calib.par_h2 / 262144.0f) *
                    (1.0f + (((float)dev->calib.par_h4 / 16384.0f) * temp_comp) +
                     (((float)dev->calib.par_h5 / 1048576.0f) * temp_comp * temp_comp))));
    var3 = (float)dev->calib.par_h6 / 16384.0f;
    var4 = (float)dev->calib.par_h7 / 2097152.0f;
    calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
    if (calc_hum > 100.0f)
    {
        calc_hum = 100.0f;
    }
    else if (calc_hum < 0.0f)
    {
        calc_hum = 0.0f;
    }

    return calc_hum;
}

float BME680::calc_gas_resistance_low(BME680 *dev, uint16_t gas_res_adc, uint8_t gas_range) {
    float calc_gas_res;
    float var1;
    float var2;
    float var3;
    float gas_res_f = gas_res_adc;
    float gas_range_f = (1U << gas_range); /*lint !e790 / Suspicious truncation, integral to float */
    const float lookup_k1_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f, 0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
    };
    const float lookup_k2_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f, -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    var1 = (1340.0f + (5.0f * dev->calib.range_sw_err));
    var2 = (var1) * (1.0f + lookup_k1_range[gas_range] / 100.0f);
    var3 = 1.0f + (lookup_k2_range[gas_range] / 100.0f);
    calc_gas_res = 1.0f / (float)(var3 * (0.000000125f) * gas_range_f * (((gas_res_f - 512.0f) / var2) + 1.0f));

    return calc_gas_res;
}

Coroutine BME680::measure(int spi) {
	auto &buffer = this->buffer;

	// allocate header for read command
	buffer.headerResize(1);

	while (true) {
		// switch to register bank 0 when in SPI mode
		if (spi) {
			buffer.header<uint8_t>() = WRITE(BME68X_REG_MEM_PAGE);
			buffer[0] = 0;
			co_await buffer.write(1);
		}

		// reset
		buffer.header<uint8_t>() = WRITE(BME68X_REG_SOFT_RESET);
		buffer[0] = BME68X_SOFT_RESET_CMD;
		co_await buffer.write(1);

		// wait
		co_await this->loop.sleep(5ms);

		// read chip id and check if it is ok
		buffer.header<uint8_t>() = READ(BME68X_REG_CHIP_ID);
		co_await buffer.read(1);
		if (this->buffer[0] == BME68X_CHIP_ID)
			break;

		// wait and try again
		co_await this->loop.sleep(1s);
	}

	// read calibration parameters
	uint8_t coeff_array[BME68X_LEN_COEFF_ALL];
	buffer.header<uint8_t>() = READ(BME68X_REG_COEFF1);
	co_await buffer.readData(coeff_array, BME68X_LEN_COEFF1);
	buffer.header<uint8_t>() = READ(BME68X_REG_COEFF2);
	co_await buffer.readData(&coeff_array[BME68X_LEN_COEFF1], BME68X_LEN_COEFF2);

	// switch to register bank 1 when in SPI mode
	if (spi) {
		buffer.header<uint8_t>() = WRITE(BME68X_REG_MEM_PAGE);
		buffer[0] = 1 << 4;
		co_await buffer.write(1);
	}

	buffer.header<uint8_t>() = READ(BME68X_REG_COEFF3);
	co_await buffer.readData(&coeff_array[BME68X_LEN_COEFF1 + BME68X_LEN_COEFF2], BME68X_LEN_COEFF3);

	get_calib_data(this, coeff_array);

	// set humidity oversampling
	buffer.header<uint8_t>() = WRITE(BME68X_REG_CTRL_HUM);
	int i = 0;
	buffer[i++] = this->regCtrlHum;// int(this->config.humidityOversampling); // spi_3w_int_en stays off

	// set temperature and pressure oversampling
	//buffer[i++] = WRITE(BME68X_REG_CTRL_MEAS);
	//buffer[i++] = this->regCtrlMeas;

	// set iir filter for temperature and pressure
	buffer[i++] = WRITE(BME68X_REG_CONFIG);
	buffer[i++] = this->regConfig;

	// set heater resistance
	buffer[i++] = BME68X_REG_RES_HEAT0;
	buffer[i++] = calc_res_heat(this, this->heaterTemperature);

	// set heater duration in milliseconds
	buffer[i++] = BME68X_REG_GAS_WAIT0;
	buffer[i++] = this->regGasWait; // calc_gas_wait(this->config.heaterDuration.value);

	// set run gas flag and profile index
	buffer[i++] = WRITE(BME68X_REG_CTRL_GAS_1);
	buffer[i++] = (1 << 4) | 0;

	// write parameters
	co_await buffer.write(i);


	while (true) {
		// start measurement
		buffer.header<uint8_t>() = WRITE(BME68X_REG_CTRL_MEAS);
		buffer[0] = this->regCtrlMeas | 1; // forced mode
		co_await buffer.write(1);

		// wait until measurement is ready
		co_await this->loop.sleep(1s);

		// read measurements
		buffer.header<uint8_t>() = READ(0x1D);
		co_await buffer.read(15);
		{
			// subtract base address so that we can use register addresses
			uint8_t const *buff = buffer.data() - 0x1D;

			// state (5.3.5 Status registers)
			bool newData = (buff[0x1D] & (1 << 7)) != 0;
			bool measuring = (buff[0x1D] & (1 << 5)) != 0; // all values
			bool gasMeasuring = (buff[0x1D] & (1 << 6)) != 0; // only gas
			int gasMeasurementIndex = buff[0x1D] & 0x0f;
			bool gasValid = (buff[0x2B] & (1 << 5)) != 0;
			bool gasHeaterStable = (buff[0x2B] & (1 << 4)) != 0;

			// measurements (5.3.4 Data registers)
			uint32_t temp_adc = buff[0x22] * 4096 | buff[0x23] * 16 | (buff[0x24] >> 4);
			uint32_t pres_adc = buff[0x1F] * 4096 | buff[0x20] * 16 | (buff[0x21] >> 4);
			uint16_t hum_adc = buff[0x25] * 256 | buff[0x26];
			uint16_t gas_res_adc = buff[0x2A] * 4 | (buff[0x2B] >> 6);
			uint8_t gas_range = buff[0x2B] & 0x0f;
			if (newData) {
				++this->frameNumber;
				this->values[0] = calc_temperature(this, temp_adc);
				this->values[1] = calc_humidity(this, hum_adc);
				this->values[2] = calc_pressure(this, pres_adc) * 0.01f;
				this->values[3] = calc_gas_resistance_low(this, gas_res_adc, gas_range);

				// notify that new data is available
				this->changeTasks.doAll();
			}
		}

		co_await this->loop.sleep(10s);
	}
}

} // namespace coco

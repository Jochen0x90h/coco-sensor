#pragma once

#include <coco/SSD130x.hpp>
#include <coco/BME680.hpp>
#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/SpiMaster_SPI_DMA.hpp>
#include <coco/platform/OutputPort_GPIO.hpp>
#include <coco/board/config.hpp>


using namespace coco;
using namespace coco::literals;

constexpr int DISPLAY_WIDTH = 128;
constexpr int DISPLAY_HEIGHT = 64;
constexpr SSD130x::Flags DISPLAY_FLAGS = SSD130x::Flags::UG_2864ASWPG01_SPI;

static const OutputPort_GPIO::Config outConfig[] {
	{gpio::PB(10), true, gpio::Config::MEDIUM, true}, // reset (CN9 7)
};

/**
 * Drivers for BME680DisplayTest with UG_2864ASWPG01 and BME680 connected via SPI
 *
 * Use pcb/Display or other SSD1309/SPI display module and connect it to the nucleo board like this:
 * GND -> CN5 7
 * 3V3 -> CN6 4
 * 12V -> CN6 5 (the nucleo board provides only 5V which is sufficient for testing)
 * SCK -> CN9 4
 * MOSI -> CN9 5
 * DC -> CN9 8
 * CS -> CN5 1
 * Reset: CN9 7 (or reset the display module by hand by connecting RESET to GND, then reset the nucleo board)
 *
 * Connect BME680 module and connect it to the nucleo board like this:
 * 3V3: CN7 16
 * GND: CN10 20
 * SCL: CN10 30
 * SDA: CN10 26
 * SDO: CN10 28
 * CS -> CN10 24
 */
struct Drivers {
	Loop_TIM2 loop{APB1_TIMER_CLOCK, Loop_TIM2::Mode::POLL};

	using SpiMaster = SpiMaster_SPI_DMA;

	// SPI for display
	SpiMaster displaySpi{loop,
		gpio::PB(3, 5), // SPI1 SCK (CN9 4) (don't forget to lookup the alternate function number in the data sheet!)
		gpio::DISCONNECTED, // no MISO, send only
		gpio::PB(5, 5), // SPI1 MOSI (CN9 5)
		gpio::PA(8), // DC (CN9 8)
		spi::SPI1_INFO,
		dma::DMA1_CH1_CH2_INFO,
		SpiMaster::ClockConfig::DIV128 | SpiMaster::ClockConfig::PHA1_POL1};
	SpiMaster::Channel displayChannel{displaySpi, gpio::PA(9), true}; // CS (CN5 1)
	SpiMaster::Buffer<DISPLAY_WIDTH * DISPLAY_HEIGHT / 8> displayBuffer{displayChannel};

	// reset pin of display
	OutputPort_GPIO resetPin{outConfig};
	AwaitableCoroutine resetDisplay() {
		resetPin.set(1, 1);
		co_await loop.sleep(10ms);
		resetPin.set(0, 1);
		co_return;
	}

	// SPI for sensor
	SpiMaster sensorSpi{loop,
		gpio::PB(13, 5), // SPI2 SCK (CN10 30)
		gpio::PB(14, 5), // SPI2 MISO (CN10 28)
		gpio::PB(15, 5), // SPI2 MOSI (CN10 26)
		spi::SPI2_INFO,
		dma::DMA1_CH3_CH4_INFO,
		SpiMaster::ClockConfig::DIV128 | SpiMaster::ClockConfig::PHA0_POL0};
	SpiMaster::Channel sensorChannel{sensorSpi, gpio::PB(1), true}; // CS (CN10 24)
	SpiMaster::Buffer<BME680::BUFFER_SIZE> sensorBuffer{sensorChannel};

	// sensor
	BME680 sensor{loop, sensorBuffer, {
		BME680::Mode::SPI,
		BME680::Oversampling::X2,
		BME680::Oversampling::X2,
		BME680::Filter::C7,
		BME680::Oversampling::X2,
		200, // gas sensor temperature
		30ms} // according to data sheet, 20-30ms are needed to reach target temperature
	};
};

Drivers drivers;

extern "C" {
void DMA1_Channel1_IRQHandler() {
	drivers.displaySpi.DMAx_Rx_IRQHandler();
}
void DMA1_Channel3_IRQHandler() {
	drivers.sensorSpi.DMAx_Rx_IRQHandler();
}
}

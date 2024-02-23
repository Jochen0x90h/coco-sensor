#pragma once

#include <coco/SSD130x.hpp>
#include <coco/platform/SSD130x_emu.hpp>
#include <coco/platform/Newline_emu.hpp>
#include <coco/platform/Sensor_emu.hpp>


using namespace coco;

constexpr int DISPLAY_WIDTH = 128;
constexpr int DISPLAY_HEIGHT = 64;
constexpr SSD130x::Flags DISPLAY_FLAGS = SSD130x::Flags::SSD1309 | SSD130x::Flags::SPI;

const Sensor_emu::Config sensorConfig[] = {
	{20.0f, -20.0f, 120.0f, 0.5f, -1, "°C"},
	{50.0f, 5.0f, 95.0f, 0.5f, -1, "%"},
	{1000.0f, 700.0f, 1300.0f, 1.0f, 0, "hPa"},
	{10.0f, 5.0f, 20.0f, 0.5f, -1, "Ω"},
};

// drivers for ButtonsTest
struct Drivers {
	Loop_emu loop;

	SSD130x_emu displayBuffer{loop, DISPLAY_WIDTH, DISPLAY_HEIGHT};
	AwaitableCoroutine resetDisplay() {co_return;}
	Newline_emu newline{loop};

	Sensor_emu sensor{loop, sensorConfig, 100};
};

Drivers drivers;

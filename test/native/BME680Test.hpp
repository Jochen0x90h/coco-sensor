#pragma once

#include <coco/SSD130x.hpp>
#include <coco/platform/Sensor_native.hpp>


using namespace coco;

constexpr int DISPLAY_WIDTH = 128;
constexpr int DISPLAY_HEIGHT = 64;
constexpr SSD130x::Flags DISPLAY_FLAGS = SSD130x::Flags::SSD1309 | SSD130x::Flags::SPI;

const Sensor_native::Config sensorConfig[] = {
	{20.0f, 3.0},
	{50.0f, 5.0f},
	{1000.0f, 20.0f},
	{10.0f, 1.0f},
};

// drivers for ButtonsTest
struct Drivers {
	Loop_native loop;

	Sensor_native sensor{loop, sensorConfig, 500ms};
};

Drivers drivers;

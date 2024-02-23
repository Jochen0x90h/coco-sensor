#include <BME680DisplayTest.hpp>
#include <coco/SSD130x.hpp>
#include <coco/font/tahoma8pt1bpp.hpp>
#include <coco/Sensor.hpp>
#include <coco/Loop.hpp>
#include <coco/StreamOperators.hpp>
#include <coco/StringBuffer.hpp>
//#include <coco/debug.hpp>


using namespace coco;


struct Info {
	// number of decimals to display (negative to disalbe suppression of trailing zeros)
	int decimalCount;

	// unit of measurement
	String unit;
};

constexpr Info infos[] = {
	{-1, "°C"},
	{-1, "%"},
	{0, "hPa"},
	{-1, "Ω"}
};

// check if sensor values have changed
bool changed(Sensor &sensor, float *values) {
	float values2[std::size(infos)];
	sensor.get(values2);

	for (size_t i = 0; i < std::size(infos); ++i) {
		if (values[i] != values2[i])
			return true;
	}
	return false;
}


Coroutine test(Loop &loop, Buffer &displayBuffer, Sensor &sensor) {
	SSD130x display(displayBuffer, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_FLAGS);
	co_await drivers.resetDisplay();
	co_await display.init();
	co_await display.enable();

	while (true) {
		float values[std::size(infos)];
		sensor.get(values);

		Bitmap bitmap = display.bitmap();
		bitmap.clear();
		//bitmap.drawText(0, 0, tahoma8pt1bpp, "Hello World!");

		for (size_t i = 0; i < std::size(infos); ++i) {
			StringBuffer<32> b;
			auto &info = infos[i];
			b << flt(values[i], info.decimalCount) << info.unit;
			bitmap.drawText(0, i * (tahoma8pt1bpp.height + 2), tahoma8pt1bpp, b);
		}

		co_await display.display();

		// wait for change of sensor values if the values haven't changed already
		if (!changed(sensor, values))
			co_await sensor.changed();
	}
}

int main() {
	test(drivers.loop, drivers.displayBuffer, drivers.sensor);

	drivers.loop.run();
}

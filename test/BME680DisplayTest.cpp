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


Coroutine test(Loop &loop, Buffer &displayBuffer, Sensor &sensor) {
	SSD130x display(displayBuffer, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_FLAGS);
	co_await drivers.resetDisplay();
	co_await display.init();
	co_await display.enable();

	int frameNumber = 0;
	while (true) {
		// get measured values
		float values[std::size(infos)];
		int nextFrameNumber;
		while ((nextFrameNumber = sensor.get(values)) == frameNumber)
			co_await sensor.changed();
		frameNumber = nextFrameNumber;

		// draw values onto display
		Bitmap bitmap = display.bitmap();
		bitmap.clear();
		for (size_t i = 0; i < std::size(infos); ++i) {
			StringBuffer<32> b;
			auto &info = infos[i];
			b << flt(values[i], info.decimalCount) << info.unit;
			bitmap.drawText(0, i * (tahoma8pt1bpp.height + 2), tahoma8pt1bpp, b);
		}
		co_await display.display();
	}
}

int main() {
	test(drivers.loop, drivers.displayBuffer, drivers.sensor);

	drivers.loop.run();
}

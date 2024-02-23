#include <BME680Test.hpp>
#include <coco/Sensor.hpp>
#include <coco/Loop.hpp>
//#include <coco/debug.hpp>


using namespace coco;

const String units[] = {
	"°C",
	"%",
	"hPa",
	"Ω",
};

Coroutine test(Loop &loop, Sensor &sensor) {
	while (true) {
		co_await sensor.changed();

		float values[4];
		sensor.get(values);

		for (size_t i = 0; i < std::size(values); ++i) {
			std::cout << values[i] << units[i] << std::endl;
		}
	}
}

int main() {
	test(drivers.loop, drivers.sensor);

	drivers.loop.run();
}

#include <coco/Sensor.hpp>
#include <coco/PseudoRandom.hpp>
#include <coco/platform/Loop_native.hpp>
#include <string>


namespace coco {

/**
 * Implementation of an emulated sensor that can measure up to 8 values
 */
class Sensor_native : public Sensor {
public:
	struct Config {
		// initial value of emulated sensor
		float initialValue;

		// deviation used for random number generator
		float deviation;
	};

	/**
		Constructor
		@param loop event loop
		@param interval emulated measurement interval
	*/
	Sensor_native(Loop_native &loop, Array<const Config> config, Milliseconds<> interval);
	~Sensor_native() override;

	int get(const Array<float> &values) override;
	[[nodiscard]] Awaitable<> changed() override;

protected:
	void handle();

	Loop_native &loop;
	Array<const Config> config;
	Milliseconds<> interval;
	TimedTask<Callback> callback;

	int frameNumber = 0;
	float values[8];
	XorShiftRandom random;
	CoroutineTaskList<> changeTasks;
};

} // namespace coco

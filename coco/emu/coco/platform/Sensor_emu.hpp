#include <coco/Sensor.hpp>
#include <coco/platform/Loop_emu.hpp>
#include <string>


namespace coco {

/**
 * Implementation of an emulated sensor that can measure up to 8 values
 */
class Sensor_emu : public Sensor, public Loop_emu::GuiHandler {
public:
	struct Config {
		// initial value of emulated sensor
		float initialValue;

		// minimum value
		float min;

		// maximum value
		float max;

		// value step
		float step;

		// number of decimals to display (negative to disalbe suppression of trailing zeros)
		int decimalCount;

		// unit of measurement
		String unit;
	};

	/**
		Constructor
		@param loop event loop
		@param id unique id for gui
	*/
	Sensor_emu(Loop_emu &loop, Array<const Config> config, int id);
	~Sensor_emu() override;

	int get(const Array<float> &values) override;
	[[nodiscard]] Awaitable<> changed() override;

protected:
	void handle(Gui &gui) override;

	Array<const Config> config;
	int id;
	int frameNumber = 0;
	float values[8];
	CoroutineTaskList<> changeTasks;
};

} // namespace coco

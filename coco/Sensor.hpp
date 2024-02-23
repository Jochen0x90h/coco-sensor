#pragma once

#include <coco/Array.hpp>
#include <coco/Coroutine.hpp>


namespace coco {

/**
 * Interface for sensors such as temperature, humidity etc.
 */
class Sensor {
public:
	virtual ~Sensor() {}

	/**
	 * Get the current measurement values of the sensors.
	 * @param values Array of measurement values
	 * @return "Frame number", can be used to determine if new values are available
	 */
	virtual int get(const Array<float> &values) = 0;

	/**
	 * Wait until measurement values have changed
	 */
	[[nodiscard]] virtual Awaitable<> changed() = 0;
};

} // namespace coco

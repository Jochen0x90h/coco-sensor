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
	 * @param values array of measurement values
	 */
	virtual void get(const Array<float> &values) = 0;

	/**
	 * Wait until measurement values have changed
	 */
	[[nodiscard]] virtual Awaitable<> changed() = 0;
};

} // namespace coco

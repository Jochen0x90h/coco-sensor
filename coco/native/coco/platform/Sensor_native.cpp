#include "Sensor_native.hpp"


namespace coco {

Sensor_native::Sensor_native(Loop_native &loop, Array<const Config> config, Milliseconds<> interval)
	: loop(loop), config(config), interval(interval), callback(makeCallback<Sensor_native, &Sensor_native::handle>(this))
{
	assert(config.size() <= std::size(this->values));
	int count = config.size();
	for (int i = 0; i < count; ++i) {
		this->values[i] = config[i].initialValue;
	}

	loop.invoke(this->callback, interval);
}

Sensor_native::~Sensor_native() {
}

int Sensor_native::get(const Array<float> &values) {
	int count = std::min(this->config.size(), values.size());
	for (int i = 0; i < count; ++i) {
		values[i] = this->values[i];
	}
	return this->frameNumber;
}

Awaitable<> Sensor_native::changed() {
	return {this->changeTasks};
}

void Sensor_native::handle() {
	++this->frameNumber;
	int count = config.size();
	for (int i = 0; i < count; ++i) {
		auto &c = this->config[i];

		// generate random deviation
		float r = int(this->random.draw()) * 4.6566e-10 * c.deviation;

		this->values[i] = c.initialValue + r;
	}
	this->changeTasks.doAll();

	this->loop.invoke(this->callback, this->interval);
}

} // namespace coco

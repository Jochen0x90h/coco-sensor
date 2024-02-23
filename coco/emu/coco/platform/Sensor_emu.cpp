#include "Sensor_emu.hpp"
#include <coco/platform/GuiRotaryKnob.hpp>
#include <coco/StringBuffer.hpp>
#include <coco/StreamOperators.hpp>
#include <coco/platform/font/tahoma16pt8bpp.hpp>
#include <iostream>


namespace coco {

Sensor_emu::Sensor_emu(Loop_emu &loop, Array<const Config> config, int id)
	: config(config), id(id)
{
	assert(config.size() <= std::size(this->values));
	int count = config.size();
	for (int i = 0; i < count; ++i) {
		this->values[i] = config[i].initialValue;
	}

	loop.guiHandlers.add(*this);
}

Sensor_emu::~Sensor_emu() {
}

int Sensor_emu::get(const Array<float> &values) {
	int count = std::min(this->config.size(), values.size());
	for (int i = 0; i < count; ++i) {
		values[i] = this->values[i];
	}
	return this->frameNumber;
}

Awaitable<> Sensor_emu::changed() {
	return {this->changeTasks};
}

void Sensor_emu::handle(Gui &gui) {
	++this->frameNumber;
	int count = config.size();
	bool changed = false;
	for (int i = 0; i < count; ++i) {
		auto &c = this->config[i];

		// rotary knob on user interface
		auto result = gui.widget<GuiRotaryKnob>(this->id + i, 50, 0.4, false);

		if (result.delta) {
			int delta = *result.delta;
			this->values[i] = std::clamp(this->values[i] + delta * c.step, c.min, c.max);
			changed = true;
		}

		StringBuffer<16> buffer;
		buffer << flt(this->values[i], c.decimalCount) << c.unit;

		// draw text onto rotary knob
		float2 scale = float2{0.2, 0.2} / 150.0f;
		gui.drawText(tahoma16pt8bpp, this->id + i, scale, buffer);

	}
	if (changed)
		this->changeTasks.doAll();
}

} // namespace coco

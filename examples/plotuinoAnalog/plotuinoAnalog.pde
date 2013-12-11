#include <WProgram.h>
#include "../../plotuino.h"

void setup() {
	Serial.begin(115200);
	Plotuino::init(&Serial);
}

void loop() {
	Plotuino::beginTransfer(0x01);
	Plotuino::send(analogRead(0));
	Plotuino::send(analogRead(1));
	Plotuino::send(analogRead(2));
	Plotuino::send(analogRead(3));
	Plotuino::endTransfer();
}

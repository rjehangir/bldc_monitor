#include <WProgram.h>
#include "../../plotuino.h"

void setup() {
	Serial.begin(115200);
}

void loop() {
	Plotuino::beginTransfer(0x01);
	Plotuino::send(analogRead(0));
	Plotuino::send(analogRead(1));
	Plotuino::endTransfer();
}

#ifndef PLOTUINO_H
#define PLOTUINO_H

#include <WProgram.h>

namespace Plotuino {
	void init(Stream* _stream);

	void beginTransfer(uint8_t messageId);

	void send(float value);

	void endTransfer();
}
#endif

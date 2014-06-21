#ifndef COMM_H
#define COMM_H

#include <WProgram.h>

namespace Comm {
	void init(Stream* _stream);

	void beginTransfer(uint8_t messageId);

	void send(float value);

	void endTransfer();
}
#endif

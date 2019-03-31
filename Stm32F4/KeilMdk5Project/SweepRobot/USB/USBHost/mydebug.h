#pragma once
template<class SERIAL_T>
void debug_hex(SERIAL_T& pc, const uint8_t* buf, int size)
{
    for(int i = 0; i < size; i++) {
        pc.printf("%02x ", buf[i]);
        if (i%16 == 15) {
            pc.puts("\n");
        }
    }
    pc.puts("\n");
}



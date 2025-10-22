/*
 * Override RTT control block to use SEGGER's standard identifier
 * This allows J-Link RTT Viewer to auto-detect the control block
 */

#include <cstring>

#include "modm/platform/rtt/rtt.hpp"

// Override the default modm RTT control block with SEGGER's standard identifier
extern "C"
{
    // SEGGER RTT control block structure
    struct SEGGER_RTT_CB
    {
        char acID[16];          // Initialized to "SEGGER RTT"
        int MaxNumUpBuffers;    // Initialized to SEGGER_RTT_MAX_NUM_UP_BUFFERS (type. 2)
        int MaxNumDownBuffers;  // Initialized to SEGGER_RTT_MAX_NUM_DOWN_BUFFERS (type. 2)
        struct
        {
            const char* sName;  // Optional name. Standard names so far are: "Terminal", "SysView",
                                // "J-Scope_i4i4i4"
            char* pBuffer;      // Pointer to start of buffer
            unsigned int SizeOfBuffer;  // Buffer size in bytes. Note that one byte is lost, as only
                                        // (SizeOfBuffer-1) bytes can be stored in the buffer.
            unsigned int WrOff;         // Position of next item to be written by either target.
            unsigned int RdOff;  // Position of next item to be read by host. Must be volatile since
                                 // it may be modified by host.
            unsigned int Flags;  // Contains configuration flags
        } aUp[2];  // Up buffers, transferring information up from target via debug probe to host
        struct
        {
            const char* sName;  // Optional name. Standard names so far are: "Terminal", "SysView",
                                // "J-Scope_i4i4i4"
            char* pBuffer;      // Pointer to start of buffer
            unsigned int SizeOfBuffer;  // Buffer size in bytes. Note that one byte is lost, as only
                                        // (SizeOfBuffer-1) bytes can be stored in the buffer.
            unsigned int WrOff;         // Position of next item to be written by either target.
            unsigned int RdOff;  // Position of next item to be read by host. Must be volatile since
                                 // it may be modified by host.
            unsigned int Flags;  // Contains configuration flags
        } aDown[2];  // Down buffers, transferring information down from host via debug probe to
                     // target
    };

    // Buffers for SEGGER RTT
    static char _acUpBuffer[1024];
    static char _acDownBuffer[256];

    // The RTT control block
    SEGGER_RTT_CB _SEGGER_RTT = {
        "SEGGER RTT",  // ID
        2,             // Max up buffers
        2,             // Max down buffers
        {{"Terminal", _acUpBuffer, sizeof(_acUpBuffer), 0, 0, 0}, {NULL, NULL, 0, 0, 0, 0}},
        {{"Terminal", _acDownBuffer, sizeof(_acDownBuffer), 0, 0, 0}, {NULL, NULL, 0, 0, 0, 0}}};
}

namespace aruwsrc::communication::serial
{
// Simple RTT wrapper using SEGGER's control block
class SeggerRtt
{
private:
    extern "C" struct SEGGER_RTT_CB _SEGGER_RTT;

public:
    size_t write(const uint8_t* data, size_t length)
    {
        if (!data || length == 0) return 0;

        size_t written = 0;
        auto& buffer = _SEGGER_RTT.aUp[0];

        for (size_t i = 0; i < length; i++)
        {
            unsigned int wrOff = buffer.WrOff;
            unsigned int nextWrOff = (wrOff + 1) % buffer.SizeOfBuffer;

            if (nextWrOff == buffer.RdOff)
            {
                // Buffer full
                break;
            }

            buffer.pBuffer[wrOff] = data[i];
            buffer.WrOff = nextWrOff;
            written++;
        }

        return written;
    }

    bool read(uint8_t& data)
    {
        auto& buffer = _SEGGER_RTT.aDown[0];

        if (buffer.RdOff == buffer.WrOff)
        {
            return false;  // Buffer empty
        }

        data = buffer.pBuffer[buffer.RdOff];
        buffer.RdOff = (buffer.RdOff + 1) % buffer.SizeOfBuffer;
        return true;
    }
};

}  // namespace aruwsrc::communication::serial
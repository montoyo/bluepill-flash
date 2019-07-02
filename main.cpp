/*
 * Created by Nicolas "montoyo" BARBOTIN in may 2019.
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/>
 */

#include <mgpcl/SerialIO.h>
#include <mgpcl/ConsoleUtils.h>
#include <mgpcl/TextIOStream.h>
#include <mgpcl/STDIOStream.h>
#include <mgpcl/Time.h>
#include <mgpcl/FileIOStream.h>
#include <mgpcl/ProgramArgs.h>

class FlashException : public std::exception
{
public:
    FlashException() : m_reason("unknown error")
    {
    }

    FlashException(const char *reason) : m_reason(reason)
    {
    }

    const char *what() const noexcept override
    {
        return m_reason;
    }

private:
    const char *m_reason;
};

enum FlashRetCode
{
    kRC_NoError = 0,
    kRC_ArgParseError,
    kRC_CommandRequired,
    kRC_SerialOpenError,
    kRC_SerialConfigureError,
    kRC_ConnectionFailure,
    kRC_ChipMissingCommand,
    kRC_InvalidPID,
    kRC_MissingCommandParam,
    kRC_WriteInputFileOpenError,
    kRC_InvalidGoAddress,
    kRC_InternalError
};

enum FlashCommand
{
    kFC_Get = 0x00,
    kFC_GetID = 0x02,
    kFC_WriteMemory = 0x31,
    kFC_ReadMemory = 0x11,
    kFC_Erase = 0x43,
    kFC_WriteUnprotect = 0x73,
    kFC_Go = 0x21
};

static const uint8_t g_allCommands[] = { kFC_Get, kFC_GetID, kFC_WriteMemory, kFC_ReadMemory, kFC_Erase, kFC_WriteUnprotect, kFC_Go };
static const char *g_allCommandNames[] = { "get", "get id", "write memory", "read memory", "erase", "write unprotect", "go" };

static_assert(sizeof(g_allCommands) == sizeof(g_allCommandNames) / sizeof(char*), "missing command name");

class Flasher
{
public:
    Flasher() : tos(new m::STDOutputStream(m::STDHandle::HOutput))
    {
        buf = new uint8_t[256];
        progBuf = new uint8_t[256];
    }

    ~Flasher()
    {
        delete[] buf;
        delete[] progBuf;
        serial.close();
    }

    FlashRetCode main(int argc, char *argv[]);

    void readTimeout(int amnt)
    {
        double t = m::time::getTimeMs();
        uint8_t *dst = buf;

        while(amnt > 0) {
            int rd = sin->read(dst, amnt);

            if(rd < 0)
                throw FlashException("read error");
            else if(rd > 0) {
                dst += rd;
                amnt -= rd;
            }

            if(m::time::getTimeMs() - t >= 5000.0)
                throw FlashException("read timeout");
        }
    }

    void doWrite(int len)
    {
        if(sout->write(buf, len) != len)
            throw FlashException("write error");
    }

    void writeByte(uint8_t b)
    {
        buf[0] = b;
        doWrite(1);
    }

    uint8_t readByte()
    {
        readTimeout(1);
        return buf[0];
    }

    void awaitAck()
    {
        uint8_t ack = readByte();

        if(ack == 0x1F)
            throw FlashException("chip sent error code");
        else if(ack != 0x79)
            throw FlashException("chip sent invalid value while awaiting ack");
    }

    m::TextOutputStream &textOut()
    {
        return tos;
    }

    void sendCommand(FlashCommand fc)
    {
        buf[0] = static_cast<uint8_t>(fc);
        buf[1] = ~static_cast<uint8_t>(fc);

        doWrite(2);
        awaitAck();
    }

    void execWrite(uint32_t startAddr, const uint8_t *src, int len)
    {
        sendCommand(kFC_WriteMemory);
        buf[0] = static_cast<uint8_t>((startAddr & 0xFF000000U) >> 24U);
        buf[1] = static_cast<uint8_t>((startAddr & 0x00FF0000U) >> 16U);
        buf[2] = static_cast<uint8_t>((startAddr & 0x0000FF00U) >>  8U);
        buf[3] = static_cast<uint8_t>((startAddr & 0x000000FFU) >>  0U);
        buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];

        doWrite(5);
        awaitAck();

        uint8_t checksum = static_cast<uint8_t>(len - 1);
        writeByte(checksum);

        while(len > 0) {
            int wr = sout->write(src, len);
            if(wr <= 0)
                throw FlashException("failed to write");

            for(int i = 0; i < wr; i++)
                checksum ^= src[i];

            src += wr;
            len -= wr;
        }

        writeByte(checksum);
        awaitAck();
    }

    void execRead(uint32_t startAddr, int len)
    {
        sendCommand(kFC_ReadMemory);
        buf[0] = static_cast<uint8_t>((startAddr & 0xFF000000U) >> 24U);
        buf[1] = static_cast<uint8_t>((startAddr & 0x00FF0000U) >> 16U);
        buf[2] = static_cast<uint8_t>((startAddr & 0x0000FF00U) >>  8U);
        buf[3] = static_cast<uint8_t>((startAddr & 0x000000FFU) >>  0U);
        buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
        doWrite(5);
        awaitAck();

        buf[0] = static_cast<uint8_t>(len - 1);
        buf[1] = ~buf[0];
        doWrite(2);
        awaitAck();

        readTimeout(len);
    }

    void go(uint32_t startAddr)
    {
        sendCommand(kFC_Go);
        buf[0] = static_cast<uint8_t>((startAddr & 0xFF000000U) >> 24U);
        buf[1] = static_cast<uint8_t>((startAddr & 0x00FF0000U) >> 16U);
        buf[2] = static_cast<uint8_t>((startAddr & 0x0000FF00U) >>  8U);
        buf[3] = static_cast<uint8_t>((startAddr & 0x000000FFU) >>  0U);
        buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
        doWrite(5);
        awaitAck();
    }

private:
    m::TextOutputStream tos;
    m::SerialPort serial;
    m::SSharedPtr<m::SerialOutputStream> sout;
    m::SSharedPtr<m::SerialInputStream> sin;
    uint8_t *buf;
    uint8_t *progBuf;
};

static int readFully(m::FileInputStream &fis, uint8_t *dst, int amnt)
{
    int total = 0;

    while(total < amnt) {
        int rd = fis.read(dst + total, amnt - total);
        if(rd <= 0)
            break;

        total += rd;
    }

    return total;
}

FlashRetCode Flasher::main(int argc, char *argv[])
{
    m::ProgramArgs pargs(argc, const_cast<const char**>(argv));

    m::ArgDescriptor &outDev     = pargs.add("--device", m::kAT_Single).addAlias("-d").setOptional(false).setHelpText("Device to program (serial port)");
    m::ArgDescriptor &noErase    = pargs.add("--no-erase", m::kAT_Switch).addAlias("-n").setHelpText("Disable flash erasing when using the \"write\" command");
    m::ArgDescriptor &noVerify   = pargs.add("--no-verify", m::kAT_Switch).addAlias("-u").setHelpText("Disable verifying when using the \"write\" command");
    m::ArgDescriptor &noPIDCheck = pargs.add("--no-pid-check", m::kAT_Switch).addAlias("-p").setHelpText("Disable PID checking");
    m::ArgDescriptor &baudRate   = pargs.add("--baud-rate", m::kAT_Single).addAlias("-s").setNumeric().setDefault(9600).setHelpText("Baud rate (default is 9600)");
    m::ArgDescriptor &helpSwitch = pargs.addHelpSwitch("--help").addAlias("-h").setHelpText("Shows this screen");
    pargs.setHelpHeader("bluepill-flash tool by Nicolas BARBOTIN\n=======================================\nUsage: bluepill-flash --device /dev/ttyDEV command1 command2 ...");
    pargs.setHelpIgnoresRequirements(true);
    pargs.setAcceptsRemainingArgs(true);

    if(pargs.parse() != m::kAPE_NoError) {
        tos << pargs.errorString() << m::eol;
        return kRC_ArgParseError;
    }

    if(helpSwitch.isSet()) {
        tos << m::eol << "Available commands:" << m::eol;
        tos << " - write binary.bin: writes binary.bin at 0x8000000 (beginning of flash)" << m::eol;
        tos << " - erase           : erases the flash" << m::eol;
        tos << " - write-unprotect : disable write protection" << m::eol;
        tos << " - start           : shorthand for \"go 0x8000000\"" << m::eol;
        tos << " - go address      : jumps to specified address" << m::eol;

        return kRC_NoError;
    }

    if(!pargs.hasRemainingArgs()) {
        tos << "You must specify at least one command! Try \"--help\" for a list of available commands." << m::eol;
        return kRC_CommandRequired;
    }

    if(!serial.open(outDev.value().asString())) {
        tos << "Failed to open serial port. Make sure that you have sufficient permissions." << m::eol;
        return kRC_SerialOpenError;
    }

    //Configure
    serial.setArduinoConfig(static_cast<m::SerialPort::BaudRate>(baudRate.value().asInt()));
    serial.setParity(m::SerialPort::kP_Even);

    if(!serial.applyConfig(true, 600)) {
        tos << "Failed to configure serial port. The selected baud rate might not be supported by your OS." << m::eol;
        serial.close();
        return kRC_SerialConfigureError;
    }

    sout = serial.outputStream<m::RefCounter>();
    sin = serial.inputStream<m::RefCounter>();

#ifdef MGPCL_WIN
    //This is required because for some reasons I can't figure out, SetCommState sends garbage (0xFF without parity bits)
    //through serial when applying configuration (i.e. calling SetCommState)

    m::console::setBackgroundColor(m::ConsoleColor::kCC_White);
    m::console::setTextColor(m::ConsoleColor::kCC_Red);
    tos << "!!! Windows workaround: please hit reset on the bluepill and then press any key here !!!" << m::eol;
    m::console::resetColor();

    {
        m::STDInputStream is(m::STDHandle::HInput);
        uint8_t tmp;
        is.read(&tmp, 1);
    }
#endif

    //Connect
    writeByte(0x7F);

    try {
        awaitAck();
    } catch(FlashException &ex) {
        tos << "Could not establish a connection to the chip. Common mistakes:" << m::eol;
        tos << " - Wrong baud rate" << m::eol;
        tos << " - Bootloader not selected (BOOT0 must be 1 and BOOT1 must be 0)" << m::eol;
        tos << " - You've swapped TX & RX pins" << m::eol;
        tos << " - You forgot to reset the chip" << m::eol;
        tos << " - Bootloader was overwritten with dank memes" << m::eol;
        tos << m::eol << "Underlying error: " << ex.what() << m::eol;

        return kRC_ConnectionFailure;
    }

    tos << "Connected." << m::eol;

    //Check available commands
    sendCommand(kFC_Get);
    uint8_t getLength = readByte();
    readTimeout(getLength + 1);
    const uint8_t blv = buf[0];
    awaitAck();

    tos << "Bootloader version " << static_cast<int>((blv & 0xF0U) >> 4U) << '.' << static_cast<int>(blv & 0x0FU) << m::eol;
    tos << static_cast<int>(getLength) << " command availables." << m::eol;

    for(int i = 0; i < sizeof(g_allCommands); i++) {
        bool found = false;

        for(int j = 1; j < getLength + 1; j++) {
            if(buf[j] == g_allCommands[i]) {
                found = true;
                break;
            }
        }

        if(!found) {
            tos << "Your chip is missing the \"" << g_allCommandNames[i] << "\" command (" << g_allCommands[i] << "). Cannot continue." << m::eol;
            return kRC_ChipMissingCommand;
        }
    }

    if(!noPIDCheck.isSet()) {
        //Check product ID
        sendCommand(kFC_GetID);
        getLength = readByte();
        readTimeout(getLength + 1);
        const uint8_t pidHigh = buf[0];
        awaitAck();

        bool validPID = false;
        if(getLength == 1) {
            uint16_t pid = static_cast<uint16_t>(pidHigh) << 8;
            pid |= static_cast<uint16_t>(buf[1]);

            validPID = (pid == 0x410);
        }

        if(!validPID) {
            tos << "This is not an medium-density STM32F10 chip. This software was meant to work with the blue pill and might not be compatible with your chip." << m::eol;
            tos << "If you still want to proceed, you can use the \"--no-pid-check\" command line switch, at your own risks." << m::eol;
            return kRC_InvalidPID;
        }
    }

    //Do what the user asked
    for(int i = pargs.remainingArgsBegin(); i < pargs.valueCount(); i++) {
        const m::String &action = pargs.value(i).asString();

        if(action.equalsIgnoreCase("write")) {
            if(i + 1 >= pargs.valueCount()) {
                tos << "Missing input filename for \"write\" command" << m::eol;
                return kRC_MissingCommandParam;
            }

            m::FileInputStream fis;
            if(fis.open(pargs.value(++i).asString()) != m::FileInputStream::kOE_Success) {
                tos << "Failed to open input file." << m::eol;
                return kRC_WriteInputFileOpenError;
            }

            fis.seek(0, m::SeekPos::End);
            const uint32_t totalSize = static_cast<uint32_t>(fis.pos());
            fis.seek(0, m::SeekPos::Beginning);

            if(!noErase.isSet()) {
                sendCommand(kFC_Erase);
                buf[0] = 0xFF;
                buf[1] = 0x00;
                doWrite(2);
                awaitAck();
                tos << "Flash erased." << m::eol;
            }

            uint32_t addr = 0x08000000U;
            bool stop = false;
            uint32_t globalPos = 0;

            while(!stop) {
                int rd = readFully(fis, progBuf, 256);
                if(rd < 256)
                    stop = true;

                globalPos += static_cast<uint32_t>(rd);
                const uint32_t progress = globalPos * 100 / totalSize;
                tos << "\rFlashing... " << progress << '%';
                tos.flush();

                if(rd % 4 != 0) {
                    int toAdd = 4 - rd % 4;

                    for(int i = 0; i < toAdd; i++)
                        progBuf[rd++] = 0;
                }

                mAssert(rd % 4 == 0 && rd <= 256, "invalid size");
                execWrite(addr, progBuf, rd);

                if(!noVerify.isSet()) {
                    execRead(addr, rd);

                    if(m::mem::cmp(progBuf, buf, static_cast<size_t>(rd)) != 0)
                        throw FlashException("verify failed (read != written)");
                }

                addr += static_cast<uint32_t>(rd);
            }

            tos << "\rFlashing... 100%" << m::eol;
        } else if(action.equalsIgnoreCase("erase")) {
            sendCommand(kFC_Erase);
            buf[0] = 0xFF;
            buf[1] = 0x00;
            doWrite(2);
            awaitAck();
            tos << "Flash erased." << m::eol;
        } else if(action.equalsIgnoreCase("write-unprotect")) {
            sendCommand(kFC_WriteUnprotect);
            awaitAck();
            writeByte(0x7F);
            awaitAck();

            tos << "Flash write protection disabled" << m::eol;
        } else if(action.equalsIgnoreCase("start")) {
            tos << "Starting program..." << m::eol;
            go(0x08000000U);
        } else if(action.equalsIgnoreCase("go")) {
            if(i + 1 >= pargs.valueCount()) {
                tos << "Missing address for \"go\" command" << m::eol;
                return kRC_MissingCommandParam;
            }

            m::String addrStr(pargs.value(++i).asString());
            uint32_t addr;

            if(addrStr.toLower().startsWith("0x"))
                addr = addrStr.toUInteger(16);
            else
                addr = addrStr.toUInteger();

            if(addr == 0) {
                tos << "Invalid address for the \"go\" command" << m::eol;
                return kRC_InvalidGoAddress;
            }

            tos << "Jumping to specified address..." << m::eol;
            go(addr);
        } else
            tos << "Unrecognized command: \"" << action.raw() << "\"." << m::eol;
    }

    tos << "All done!" << m::eol;
    return kRC_NoError;
}

int main(int argc, char *argv[])
{
    m::time::initTime();
    Flasher f;

    try {
        return static_cast<int>(f.main(argc, argv));
    } catch(const FlashException &ex) {
        f.textOut() << "\rFailed to flash: " << ex.what() << m::eol;
    }

    return static_cast<int>(kRC_InternalError);
}

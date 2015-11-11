/*
 * Fadecandy driver for the Enttec DMX USB Pro.
 * 
 * Copyright (c) 2013 Micah Elizabeth Scott
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "bw_wsdevice.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "opc.h"
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>


BitWizardWSDevice::Transfer::Transfer(BitWizardWSDevice *device, void *buffer, int length)
    : transfer(libusb_alloc_transfer(0)), finished(false)
{
    libusb_fill_bulk_transfer(transfer, device->mHandle,
        OUT_ENDPOINT, (uint8_t*) buffer, length, BitWizardWSDevice::completeTransfer, this, 2000);
}

BitWizardWSDevice::Transfer::~Transfer()
{
    libusb_free_transfer(transfer);
}

BitWizardWSDevice::BitWizardWSDevice(libusb_device *device, bool verbose)
    : USBDevice(device, "BitWizard USB WS2812", verbose),
      mFoundBWStrings(false),
      mConfigMap(0)
{
    mSerialBuffer[0] = '\0';
    mSerialString = mSerialBuffer;

    // Initialize a minimal valid DMX packet
    memset(&mChannelBuffer, 0, sizeof mChannelBuffer);
    mChannelBuffer.start = START_OF_MESSAGE;
    mChannelBuffer.label = SEND_DMX_PACKET;
    mChannelBuffer.data[0] = START_CODE;
    //setChannel(1, 0);
}

BitWizardWSDevice::~BitWizardWSDevice()
{
    /*
     * If we have pending transfers, cancel them.
     * The Transfer objects themselves will be freed once libusb completes them.
     */

    for (std::set<Transfer*>::iterator i = mPending.begin(), e = mPending.end(); i != e; ++i) {
        Transfer *fct = *i;
        libusb_cancel_transfer(fct->transfer);
    }
}

bool BitWizardWSDevice::probe(libusb_device *device)
{
    /*
     * Prior to opening the device, all we can do is look for an FT245 device.
     * We'll take a closer look in probeAfterOpening(), once we can see the
     * string descriptors.
     */

    libusb_device_descriptor dd;

    if (libusb_get_device_descriptor(device, &dd) < 0) {
        // Can't access descriptor?
        return false;
    }

    // FTDI FT245
    return dd.idVendor == 0x0483 && dd.idProduct == 0xf740;
}


int BitWizardWSDevice::open()
{
    libusb_device_descriptor dd;
    int r = libusb_get_device_descriptor(mDevice, &dd);
    if (r < 0) {
        return r;
    }

    r = libusb_open(mDevice, &mHandle);
    if (r < 0) {
        return r;
    }

    /*
     * Match the manufacturer and product strings! This is the least intrusive way to
     * determine that the attached device is in fact an Enttec DMX USB Pro, since it doesn't
     * have a unique vendor/product ID.
     */

    if (dd.iManufacturer && dd.iProduct && dd.iSerialNumber) {
        char manufacturer[256];
        char product[256];

        r = libusb_get_string_descriptor_ascii(mHandle, dd.iManufacturer, (uint8_t*)manufacturer, sizeof manufacturer);
        if (r < 0) {
            return r;
        }
        r = libusb_get_string_descriptor_ascii(mHandle, dd.iProduct, (uint8_t*)product, sizeof product);
        if (r < 0) {
            return r;
        }

        mFoundBWStrings = 
	  !strcmp(manufacturer, "STMicroelectronics")
	  // && !strcmp(product, "DMX USB PRO")
	  ;
    }

    /*
     * Only go further if we have in fact found evidence that this is the right device.
     */

    if (mFoundBWStrings) {
      std::string line, s;
      int p;

      mIstream.open ("/dev/bw_ws2812");
      mOstream.open ("/dev/bw_ws2812");
      if (!mIstream.is_open()) 
	return -1;
      if (!mOstream.is_open()) 
	return -1;

      mOstream  << "\nuid\n";
      mOstream.flush ();
      while ( getline (mIstream, line) ) {
	p = line.find ("uid=");
	std::cout << "got line: " << line << " p=" << p << "\n"; 
	if (p != std::string::npos) {
	  size_t endpos = line.find_last_not_of(" \t\r");
	  if( std::string::npos != endpos )   
	    line = line.substr( 0, endpos+1);

	  mSerialString = strdup (line.substr (p+4).c_str());
	  std::cout << "got serial:  " <<  mSerialString << "\n";
	  break;
	}
      }

      if (mOstream.fail())
	std::cout << "open stream fail error.\n";
      if (mOstream.bad())
	std::cout << "open stream bad error.\n";

      std::cout << "bw detected.... " << mTypeString <<" serial: " << mSerialString << "\n";
    }

    return 0;
}


bool BitWizardWSDevice::probeAfterOpening()
{
    // By default, any device is supported by the time we get to opening it.
    return mFoundBWStrings;
}


void BitWizardWSDevice::loadConfiguration(const Value &config)
{
    mConfigMap = findConfigMap(config);
}


std::string BitWizardWSDevice::getName()
{
    std::ostringstream s;
    s << "BitWizard USB-WS2812";
    if (mSerialString[0]) {
        s << " (Serial# " << mSerialString << ")";
    }
    return s.str();
}


#if 0
void BitWizardWSDevice::setChannel(unsigned n, uint8_t value)
{
    if (n >= 1 && n <= 512) {
        unsigned len = std::max<unsigned>(mChannelBuffer.length, n + 1);
        mChannelBuffer.length = len;
        mChannelBuffer.data[n] = value;
        mChannelBuffer.data[len] = END_OF_MESSAGE;
    }
}
#endif

#if 0
void BitWizardWSDevice::submitTransfer(Transfer *fct)
{
    /*
     * Submit a new USB transfer. The Transfer object is guaranteed to be freed eventually.
     * On error, it's freed right away.
     */

    int r = libusb_submit_transfer(fct->transfer);

    if (r < 0) {
        if (mVerbose && r != LIBUSB_ERROR_PIPE) {
            std::clog << "Error submitting USB transfer: " << libusb_strerror(libusb_error(r)) << "\n";
        }
        delete fct;
    } else {
        mPending.insert(fct);
    }
}
#endif


void BitWizardWSDevice::completeTransfer(struct libusb_transfer *transfer)
{
    BitWizardWSDevice::Transfer *fct = static_cast<BitWizardWSDevice::Transfer*>(transfer->user_data);
    fct->finished = true;
}

void BitWizardWSDevice::flush()
{
    // Erase any finished transfers

  //std::cout << "Got a flush.... \n"; 
#if 0
    std::set<Transfer*>::iterator current = mPending.begin();
    while (current != mPending.end()) {
        std::set<Transfer*>::iterator next = current;
        next++;

        Transfer *fct = *current;
        if (fct->finished) {
            mPending.erase(current);
            delete fct;
        }

        current = next;
    }
#endif
}

#if 0
void BitWizardWSDevice::writeDMXPacket()
{
    /*
     * Asynchronously write an FTDI packet containing an Enttec packet containing
     * our set of DMX channels.
     *
     * XXX: We should probably throttle this so that we don't send DMX messages
     *      faster than the Enttec device can keep up!
     */

    submitTransfer(new Transfer(this, &mChannelBuffer, mChannelBuffer.length + 5));
}
#endif




void BitWizardWSDevice::writeFramebuffer()
{
  char header[] = {
    0x01, // START binary command
    0x01, // write pixels command
    0x00, // num pixels low
    0x00  // num pixels high
  };

  header[2] = NUM_PIXELS & 0xff;
  header[3] = NUM_PIXELS >> 8;

  if (!mOstream.is_open() ) {
    std::cout << "hmm. file not open???\n";
  }

  if (mOstream.fail())
    std::cout << "nowrite fail error.\n";
  if (mOstream.bad())
    std::cout << "nowrite bad error.\n";

  mOstream.write (header, sizeof(header));

  if (mOstream.fail())
    std::cout << "write fail error.\n";
  if (mOstream.bad())
    std::cout << "write bad error.\n";

  mOstream.write ((char*)mFramebuffer, NUM_PIXELS*3);
  mOstream.flush ();
}



void BitWizardWSDevice::writeMessage(const OPC::Message &msg)
{
  /*
   * Dispatch an incoming OPC command
   */
  //  std::cout  << "got a write message! ";

  switch (msg.command) {

  case OPC::SetPixelColors:
    //std::cout  << "setpixels...\n";

    opcSetPixelColors(msg);
    writeFramebuffer();
    return;

  case OPC::SystemExclusive:
    std::cout  << "sysex...\n";
    // No relevant SysEx for this device
    return;
  default: 
    std::cout  << "unhandled case...\n";
  }

  if (mVerbose) {
    std::clog << "Unsupported OPC command: " << unsigned(msg.command) << "\n";
  }
}

void BitWizardWSDevice::opcSetPixelColors(const OPC::Message &msg)
{
  /*
   * Parse through our device's mapping, and store any relevant portions of 'msg'
   * in the framebuffer.
   */

  if (!mConfigMap) {
    // No mapping defined yet. This device is inactive.
    std::cout  << "no mapping...\n";
    return;
  }

  const Value &map = *mConfigMap;
  for (unsigned i = 0, e = map.Size(); i != e; i++) {
    opcMapPixelColors(msg, map[i]);
  }
}


void BitWizardWSDevice::opcMapPixelColors(const OPC::Message &msg, const Value &inst)
{
  /*
   * Parse one JSON mapping instruction, and copy any relevant parts of 'msg'
   * into our framebuffer. This looks for any mapping instructions that we
   * recognize:
   *
   *   [ OPC Channel, OPC Pixel, Pixel Color, DMX Channel ]
   */

  unsigned msgPixelCount = msg.length() / 3;

  if (inst.IsArray() && inst.Size() == 4) {
    // Map a range from an OPC channel to our framebuffer

    const Value &vChannel  = inst[0u];
    const Value &vFirstOPC = inst[1];
    const Value &vFirstOut = inst[2];
    const Value &vCount    = inst[3];


    if (vChannel.IsUint() && vFirstOPC.IsUint() && 
	vFirstOut.IsUint() && vCount.IsUint()) {
      unsigned channel  = vChannel.GetUint();
      unsigned firstOPC = vFirstOPC.GetUint();
      unsigned firstOut = vFirstOut.GetUint();
      unsigned count    = vCount.GetUint();
      

      if (channel != msg.channel) {
	return;
      }

      // Clamping, overflow-safe
      firstOPC = std::min<unsigned>(firstOPC, msgPixelCount);
      firstOut = std::min<unsigned>(firstOut, unsigned(NUM_PIXELS));
      count = std::min<unsigned>(count, msgPixelCount - firstOPC);
      count = std::min<unsigned>(count, NUM_PIXELS - firstOut);
      
      // Copy pixels
      const uint8_t *inPtr = msg.data + (firstOPC * 3);
      unsigned outIndex = firstOut;

      //std::cout  << "mapping " << count << " pixels at " << firstOut << "...\n";
      
      while (count--) {
	uint8_t *outPtr = fbPixel(outIndex++);
	outPtr[0] = inPtr[0];
	outPtr[1] = inPtr[1];
	outPtr[2] = inPtr[2];
	inPtr += 3;
	if (outIndex >= NUM_PIXELS) return;
      }
      
      return;
    }
  }

  if (inst.IsArray() && inst.Size() == 2) {
    // Constant value

    const Value &vValue = inst[0u];
    const Value &vDMXChannel = inst[1];

    if (vValue.IsUint() && vDMXChannel.IsUint()) {
      unsigned value = vValue.GetUint();
      unsigned dmxChannel = vDMXChannel.GetUint();

      //setChannel(dmxChannel, value);
      return;
    }
  }

  // Still haven't found a match?
  if (mVerbose) {
    rapidjson::GenericStringBuffer<rapidjson::UTF8<> > buffer;
    rapidjson::Writer<rapidjson::GenericStringBuffer<rapidjson::UTF8<> > > writer(buffer);
    inst.Accept(writer);
    std::clog << "Unsupported JSON mapping instruction: " << buffer.GetString() << "\n";
  }
}

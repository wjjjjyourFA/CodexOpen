#pragma once

#include <libyuv.h>
#include <stdint.h>
#include <turbojpeg.h>

#include <cstddef>
#include <cstdlib>
#include <vector>

class JpegEnc {
 public:
  JpegEnc(int chan, int width, int height, int quality = 90);
  ~JpegEnc();

  virtual bool Init();

  virtual void Release();

  void Encode(uint8_t* yuyv, uint8_t*& jpegPtr, size_t& jpegSize);

 private:
  int m_chan;
  int m_width;
  int m_height;
  int m_quality;

  int m_stride;

  /* I420 buffer */
  unsigned char* m_yuv;
  uint8_t* m_y;
  uint8_t* m_u;
  uint8_t* m_v;

  /* turbojpeg */
  tjhandle tj;

  /* fixed jpeg buffer */
  std::vector<uint8_t> jpegBuffer;
};

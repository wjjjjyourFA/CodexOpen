#include "modules/drivers/camera/jpeg_encode.h"

JpegEnc::JpegEnc(int chan, int width, int height, int quality) {
  m_chan    = chan;
  m_width   = width;
  m_height  = height;
  m_quality = quality;

  /* SIMD stride align */

  m_stride = (m_width + 31) & ~31;

  m_yuv = nullptr;

  tj = tjInitCompress();
}

JpegEnc::~JpegEnc() {
  Release();

  if (tj) {
    tjDestroy(tj);
    tj = nullptr;
  }
}

bool JpegEnc::Init() {
  // 分配YUV420p缓冲区空间
  // m_yuv = new unsigned char[m_width * m_height * 3 / 2];
  // m_yuv = (uint8_t*)malloc(m_width * m_height * 3 / 2);
  posix_memalign((void**)&m_yuv, 64, m_width * m_height * 3 / 2);

  if (!m_yuv) {
    return false;
  }

  m_y = m_yuv;
  m_u = m_yuv + m_width * m_height;
  m_v = m_yuv + m_width * m_height * 5 / 4;

  size_t jpegMax = tjBufSize(m_width, m_height, TJSAMP_420);
  jpegBuffer.reserve(jpegMax);
  jpegBuffer.resize(jpegMax);

  return true;
}

void JpegEnc::Release() {
  if (m_yuv) {
    // delete[] m_yuv;
    free(m_yuv);
    m_yuv = nullptr;
  }
}

void JpegEnc::Encode(uint8_t* yuyv /*unsigned char *pData*/, uint8_t*& jpegPtr,
                     size_t& jpegSize) {
  if (!yuyv) {
    jpegPtr  = nullptr;
    jpegSize = 0;
    return;
  }

  // 将YUY2格式转换为I420格式（JPEG编码器要求）
  // 1920 / 1280 / 640 等常见分辨率没问题
  // libyuv::YUY2ToI420(yuyv, 2 * m_stride,  // YUY2数据及步长
  //                    m_y, m_width,  // Y分量缓冲区及步长
  //                    m_u, m_width / 2,  // U分量缓冲区及步长
  //                    m_v, m_width / 2,  // V分量缓冲区及步长
  //                    m_width, m_height);

  libyuv::YUY2ToI420(yuyv, 2 * m_width,  // YUY2数据及步长
                     m_y, m_width,  // Y分量缓冲区及步长
                     m_u, m_width / 2,  // U分量缓冲区及步长
                     m_v, m_width / 2,  // V分量缓冲区及步长
                     m_width, m_height);

  unsigned char* outBuf = jpegBuffer.data();
  unsigned long outSize = jpegBuffer.size();

  const unsigned char* planes[3] = {m_y, m_u, m_v};

  int ret = tjCompressFromYUVPlanes(tj, planes, m_width, nullptr, m_height,
                                    TJSAMP_420, &outBuf, &outSize, m_quality,
                                    TJFLAG_FASTDCT | TJFLAG_NOREALLOC);

  if (ret != 0) {
    jpegPtr  = nullptr;
    jpegSize = 0;
    return;
  }

  jpegPtr  = outBuf;
  jpegSize = outSize;
}

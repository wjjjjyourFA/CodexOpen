#include "cyber/io_bridge/dds/dds_cv_bridge.h"

#ifdef ENABLE_DDS

namespace dds {
namespace cv_bridge {

std::shared_ptr<image_dds_type> CvImage::toImageMsg() const {
  std::shared_ptr<image_dds_type> image_msg_ptr =
      std::make_shared<image_dds_type>();
  image_msg_ptr->header()       = header;
  image_msg_ptr->height()       = image.rows;
  image_msg_ptr->width()        = image.cols;
  image_msg_ptr->encoding()     = encoding;
  image_msg_ptr->is_bigendian() = false;
  image_msg_ptr->step()         = image.step1();

  image_msg_ptr->data().resize(image.total() * image.channels());
  memcpy(image_msg_ptr->data().data(), image.data,
         image_msg_ptr->data().size() * sizeof(uint8_t));

  return image_msg_ptr;
}

std::shared_ptr<compressed_image_dds_type> CvImage::toCompressedImageMsg(
    int compress_quality) const {
  std::shared_ptr<compressed_image_dds_type> compressed_image_msg_ptr =
      std::make_shared<compressed_image_dds_type>();
  compressed_image_msg_ptr->header() = header;
  compressed_image_msg_ptr->format("jpg");

  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(compress_quality);  // 压缩质量

  // 压缩图像
  cv::imencode(".jpg", image, compressed_image_msg_ptr->data(),
               compression_params);

  return compressed_image_msg_ptr;
}

CvImagePtr toCvCopy(const image_dds_type& source, const std::string& encoding) {
  int channel = source.step() / source.width();
  uint8_t type;

  switch (channel) {
    case 1:
      type = CV_8UC1;
      break;
    case 3:
      type = CV_8UC3;
      break;
    default:
      break;
  }

  std::vector<uint8_t> image_data = source.data();

  const cv::Mat recv_img(source.height(), source.width(), type,
                         image_data.data());

  CvImagePtr cv_image_ptr = std::make_shared<CvImage>(
      source.header(), source.encoding(), std::move(recv_img.clone()));

  return cv_image_ptr;
}

CvImagePtr toCvCopy(const compressed_image_dds_type& source,
                    const std::string& encoding) {
  cv::Mat decode_img;

  cv::imdecode(source.data(), cv::IMREAD_COLOR, &decode_img);

  CvImagePtr cv_image_ptr =
      std::make_shared<CvImage>(source.header(), "bgr8", decode_img);

  return cv_image_ptr;
}

}  // namespace cv_bridge
}  // namespace dds

#endif  // ENABLE_DDS
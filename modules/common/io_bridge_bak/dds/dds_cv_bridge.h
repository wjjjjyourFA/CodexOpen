#ifndef DDS_CV_BRIDGE_H
#define DDS_CV_BRIDGE_H
#include "opencv2/opencv.hpp"
#include "cyber/common/environment_conf.h"

#ifdef ENABLE_DDS
#include "dds/dds.hpp"
#include "dds/msg/Image.hpp"
#include "dds/msg/CompressedImage.hpp"

typedef dds::msg::Image image_dds_type;
typedef dds::msg::CompressedImage compressed_image_dds_type;

namespace dds {
namespace cv_bridge {

class CvImage {
 public:
  msg::Header header;  //!< ROS header
  std::string encoding;  //!< Image encoding ("mono8", "bgr8", etc.)
  cv::Mat image;  //!< Image data for use with OpenCV

  /**
   * \brief Empty constructor.
   */
  CvImage() {}

  /**
   * \brief Constructor.
   */
  CvImage(const dds::msg::Header& header, const std::string& encoding,
          const cv::Mat& image = cv::Mat())
      : header(header), encoding(encoding), image(image) {}

  /**
   * \brief Convert this message to a ROS sensor_msgs::Image message.
   *
   * The returned sensor_msgs::Image message contains a copy of the image data.
   */
  std::shared_ptr<image_dds_type> toImageMsg() const;

  /**
   * dst_format is compress the image to desire format.
   * Default value is empty string that will convert to jpg format.
   * can be: jpg, jp2, bmp, png, tif at the moment
   * support this format from opencv:
   * http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const string& filename, int flags)
   */
  std::shared_ptr<compressed_image_dds_type> toCompressedImageMsg(
      int compress_quality = 80) const;
};

typedef std::shared_ptr<CvImage> CvImagePtr;
typedef std::shared_ptr<CvImage const> CvImageConstPtr;

/**
 * \brief Convert a sensor_msgs::Image message to an OpenCV-compatible CvImage, copying the
 * image data.
 *
 * \param source   A sensor_msgs::Image message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 * If the source is 8bit and the encoding 16 or vice-versa, a scaling is applied (65535/255 and
 * 255/65535 respectively). Otherwise, no scaling is applied and the rules from the convertTo OpenCV
 * function are applied (capping): http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-convertto
 */
CvImagePtr toCvCopy(const image_dds_type& source,
                    const std::string& encoding = std::string());

CvImagePtr toCvCopy(const compressed_image_dds_type& source,
                    const std::string& encoding = std::string());

}  // namespace cv_bridge
}  // namespace dds

#endif  // ENABLE_DDS

#endif  // CV_BRIDGE_H

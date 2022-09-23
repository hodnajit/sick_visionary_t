/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 * \note
 *   Copyright (c) 2015 \n
 *   SICK AG \n\n
 *
 *****************************************************************
 *
 * \note
 *   ROS package name: sick_visionary_t_driver
 *
 * \author
 *   Author: Joshua Hampp
 *
 * \date Date of creation: 05/21/2015
 *
 *****************************************************************
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************/

#include <endian.h>

#if __BYTE_ORDER == __BIG_ENDIAN
uint64_t swapLong(uint64_t x) {
	x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
	x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
	x = (x & 0x00FF00FF00FF00FF) << 8  | (x & 0xFF00FF00FF00FF00) >> 8;
	return x;
}

#define __Swap2Bytes(val)   ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) )
#define __Swap2BytesD(val)  (val>7490?0:( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) ))
#define __Swap4Bytes(dword) ( ((dword>>24)&0x000000FF) | ((dword>>8)&0x0000FF00) | ((dword<<8)&0x00FF0000) | ((dword<<24)&0xFF000000) )
#define __Swap8Bytes(val) (swapLong(val))
#else
#define __Swap2Bytes(val) (val)
#define __Swap2BytesD(val) (val>7490?0:val)
#define __Swap4Bytes(dword) (dword)
#define __Swap8Bytes(dword) (dword)
#endif


/* This class gathers the main camera parameters. */
struct CameraParameters {
    uint32_t width, height;
    double cam2worldMatrix[4 * 4];
    double fx, fy, cx, cy, k1, k2, f2rc;

    CameraParameters(uint32_t width = 176, uint32_t height = 144,
                     double *cam2worldMatrix = NULL,
                     double fx = 146.5, double fy = 146.5, double cx = 84.4, double cy = 71.2,
                     double k1 = 0.326442, double k2 = 0.219623,
                     double f2rc = 0.0) :
            width(width), height(height),
            fx(fx), fy(fy),
            cx(cx), cy(cy),
            k1(k1), k2(k2),
            f2rc(f2rc) {
        if (cam2worldMatrix)
            memcpy(this->cam2worldMatrix, cam2worldMatrix, sizeof(this->cam2worldMatrix));
    }
};


/* Gathers methods to handle the raw data. */
class Data {
    CameraParameters cameraParams_;
    cv::Mat distance_, intensity_, confidence_;
    int numBytesPerDistanceValue_, numBytesPerIntensityValue_, numBytesPerConfidenceValue_;
    uint32_t framenumber_;
public:

    Data() : framenumber_(0) {
    }

    const cv::Mat &get_distance() const { return distance_; }

    const cv::Mat &get_intensity() const { return intensity_; }

    const cv::Mat &get_confidence() const { return confidence_; }

    uint32_t get_framenumber() const { return framenumber_; }

    const CameraParameters &getCameraParameters() const { return cameraParams_; }

    /* Get size of complete package. */
    static size_t actual_size(const char *data, const size_t size) {
        if (size < 8) return 0;
        // first 11 bytes contain some internal definitions   
        const uint32_t pkglength = ntohl(*(uint32_t *) (data + 4));

        return pkglength + 9;
    }

    /* Check if there are enough data to be parsed. */
    static bool check_header(const char *data, const size_t size) {
        if (size < 11) return false;

        const uint32_t magicword = ntohl(*(uint32_t *) (data + 0));

        //if magic word in invalid we'll read the content anyway to skip data
        return magicword != 0x02020202 || size >= actual_size(data, size);
    }

    /* Extracts necessary data segments and triggers parsing of segments. */
    bool read(const char *data, const size_t size) {
        // first 11 bytes contain some internal definitions   
        const uint32_t magicword = ntohl(*(uint32_t *) (data + 0));
        const uint32_t pkglength = ntohl(*(uint32_t *) (data + 4));
        const uint16_t protocolVersion = ntohs(*(uint16_t *) (data + 8));
        const uint8_t packetType = *(uint8_t *) (data + 10);

        rcpputils::assert_true(magicword == 0x02020202);
        rcpputils::assert_true(protocolVersion == 1);
        rcpputils::assert_true(packetType == 98);

        if (magicword != 0x02020202 || protocolVersion != 1 || packetType != 98)
            return false;

        // next four bytes an id (should equal 1) and
        // the number of segments (should be 3)       
        const uint16_t id = ntohs(*(uint16_t *) (data + 11));
        const uint16_t numSegments = ntohs(*(uint16_t *) (data + 13));
        rcpputils::assert_true(id == 1);
        rcpputils::assert_true(numSegments == 3);

        // offset and changedCounter, 4 bytes each per segment
        uint32_t offset[numSegments];
        uint32_t changedCounter[numSegments];
        for (size_t i = 0; i < (size_t) numSegments; i++) {
            offset[i] = 11 + ntohl(*(uint32_t *) (data + (i * 8 + 15)));
            changedCounter[i] = ntohl(*(uint32_t *) (data + (i * 8 + 19)));
        }

        // XML segment describes the data format
        std::string xmlSegment(data + offset[0], offset[1] - offset[0]);

        // parsing the XML in order to extract necessary image information
        parseXML(xmlSegment);

        /*self.cameraParams = \
                CameraParameters(width=myXMLParser.imageWidth,
                                 height=myXMLParser.imageHeight,
                                 cam2worldMatrix=myXMLParser.cam2worldMatrix,
                                 fx=myXMLParser.fx,fy=myXMLParser.fy,
                                 cx=myXMLParser.cx,cy=myXMLParser.cy,
                                 k1=myXMLParser.k1,k2=myXMLParser.k2,
                                 f2rc=myXMLParser.f2rc)*/

        // extracting data from the binary segment (distance, intensity
        // and confidence).
        rcpputils::assert_true(numBytesPerDistanceValue_ == 2);
        rcpputils::assert_true(numBytesPerIntensityValue_ == 2);
        rcpputils::assert_true(numBytesPerConfidenceValue_ == 2);

        distance_ = cv::Mat(cameraParams_.height, cameraParams_.width, CV_16UC1);
        intensity_ = cv::Mat(cameraParams_.height, cameraParams_.width, CV_16UC1);
        confidence_ = cv::Mat(cameraParams_.height, cameraParams_.width, CV_16UC1);

        getData(data + offset[1], size - offset[1]);

        return true;
    }

private:
    /* Parse method needs the XML segment as string input. */
    bool parseXML(const std::string &xmlString) {
        numBytesPerConfidenceValue_ = numBytesPerIntensityValue_ = numBytesPerConfidenceValue_ = -1;

        boost::property_tree::ptree pt;
        std::istringstream ss(xmlString);
        try {
            boost::property_tree::xml_parser::read_xml(ss, pt);
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "failed to parse response (XML malformed)\ncontent: %s", xmlString.c_str());
            return false;
        }

        boost::property_tree::ptree ds = pt.get_child(
                "SickRecord.DataSets.DataSetDepthMap.FormatDescriptionDepthMap.DataStream");
        cameraParams_.width = ds.get<uint32_t>("Width");
        cameraParams_.height = ds.get<uint32_t>("Height");

        int i = 0;
        BOOST_FOREACH(const boost::property_tree::ptree::value_type &item, ds.get_child("CameraToWorldTransform")) {
                        if (i < 16)
                            cameraParams_.cam2worldMatrix[i] = item.second.get_value<double>();
                        ++i;
                    }
        rcpputils::assert_true(i == 16);

        cameraParams_.fx = ds.get<double>("CameraMatrix.FX");
        cameraParams_.fy = ds.get<double>("CameraMatrix.FY");
        cameraParams_.cx = ds.get<double>("CameraMatrix.CX");
        cameraParams_.cy = ds.get<double>("CameraMatrix.CY");

        cameraParams_.k1 = ds.get<double>("CameraDistortionParams.K1");
        cameraParams_.k2 = ds.get<double>("CameraDistortionParams.K2");

        cameraParams_.f2rc = ds.get<double>("FocalToRayCross");

        // extract data format from XML (although it does not change)
        std::string data_type;

        data_type = ds.get<std::string>("Distance", "");
        boost::algorithm::to_lower(data_type);
        if (data_type != "") {
            rcpputils::assert_true(data_type == "uint16");
            numBytesPerDistanceValue_ = 2;
        }

        data_type = ds.get<std::string>("Intensity", "");
        boost::algorithm::to_lower(data_type);
        if (data_type != "") {
            rcpputils::assert_true(data_type == "uint16");
            numBytesPerIntensityValue_ = 2;
        }

        data_type = ds.get<std::string>("Confidence", "");
        boost::algorithm::to_lower(data_type);
        if (data_type != "") {
            rcpputils::assert_true(data_type == "uint16");
            numBytesPerConfidenceValue_ = 2;
        }

        return true;
    }

    void getData(const char *data, const size_t size) {
        rcpputils::assert_true(size >= 14);
        if (size < 14) {
            RCLCPP_WARN(get_logger(), "malformed data (1): %d<14", (int) size);
            return;
        }
        const size_t numBytesDistance = (numBytesPerDistanceValue_ > 0) ? distance_.total() * numBytesPerDistanceValue_
                                                                        : 0;
        const size_t numBytesIntensity = (numBytesPerIntensityValue_ > 0) ? intensity_.total() *
                                                                            numBytesPerIntensityValue_ : 0;
        const size_t numBytesConfidence = (numBytesPerConfidenceValue_ > 0) ? confidence_.total() *
                                                                              numBytesPerConfidenceValue_ : 0;

        // the binary part starts with entries for length, a timestamp
        // and a version identifier
        size_t offset = 0;
        const uint32_t length = __Swap4Bytes(*(uint32_t *) (data + offset));
        const uint64_t timeStamp = __Swap8Bytes(*(uint64_t *) (data + offset + 4));
        const uint16_t version = __Swap2Bytes(*(uint16_t *) (data + offset + 12));

        offset += 14; //calcsize('>IQH')

        if (length > size) {
            RCLCPP_WARN(get_logger(), "malformed data (2): %d>%d", (int) length, (int) size);
            return;
        }

        if (version > 1) {
            // more frame information follows in this case: frame number, data quality, device status ('<IBB')
            framenumber_ = ntohl(*(uint32_t *) (data + offset));
            const uint8_t dataQuality = *(uint8_t *) (data + offset + 4);
            const uint8_t deviceStatus = *(uint8_t *) (data + offset + 5);
            offset += 6;
        } else {
            ++framenumber_;
        }

        size_t end = offset + numBytesDistance + numBytesIntensity + numBytesConfidence; // calculating the end index
        //wholeBinary = binarySegment[start:end] // whole data block
        //distance = wholeBinary[0:numBytesDistance] // only the distance
        // data (as string)

        rcpputils::assert_true(end <= size);

        if (numBytesDistance > 0) {
            for (size_t i = 0; i < distance_.total(); i++)
                distance_.at<uint16_t>(i) = __Swap2BytesD(*(uint16_t *) (data + offset + i * 2));
            offset += numBytesDistance;
        } else {
            distance_.setTo(0);
        }
        if (numBytesIntensity > 0) {
            for (size_t i = 0; i < intensity_.total(); i++)
                intensity_.at<uint16_t>(i) = __Swap2Bytes(*(uint16_t *) (data + offset + i * 2));
            offset += numBytesIntensity;
        } else {
            intensity_.setTo(std::numeric_limits<uint16_t>::max() / 2);
        }
        if (numBytesConfidence > 0) {
            for (size_t i = 0; i < confidence_.total(); i++)
                confidence_.at<uint16_t>(i) = __Swap2Bytes(*(uint16_t *) (data + offset + i * 2));
            offset += numBytesConfidence;
        } else {
            confidence_.setTo(std::numeric_limits<uint16_t>::max());
        }

        // data end with a 32byte (unused) CRC field and a copy of the length byte
        const uint32_t unusedCrc = __Swap4Bytes(*(uint32_t *) (data + offset));
        const uint32_t lengthCopy = __Swap4Bytes(*(uint32_t *) (data + offset + 4));

        if (length != lengthCopy)
            RCLCPP_WARN(get_logger(), "check failed %d!=%d", (int) length, (int) lengthCopy);
        rcpputils::assert_true(length == lengthCopy);
    }

    static rclcpp::Logger get_logger() {
        return rclcpp::get_logger("Data");
    }

};

#undef __Swap2Bytes

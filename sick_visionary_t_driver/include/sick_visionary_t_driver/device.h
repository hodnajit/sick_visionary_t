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

class CoLaFrame {
	std::vector<char> buffer_;

	uint8_t checksum() {
		uint8_t c=0;
		for(size_t i=8; i<buffer_.size(); i++)
			c^=buffer_[i];
		return c;
	}

public:
	CoLaFrame() : buffer_(8, (char)0) {
	}

	CoLaFrame(const char *str) : buffer_(8+strlen(str), (char)0) {
		int i=8;
		while(*str)
			buffer_[i++] = *(str++);
	}

	std::vector<char> finish() {
		char *data = &buffer_[0];
		*(uint32_t*)(data+0) = htonl(0x02020202);
		*(uint32_t*)(data+4) = htonl(buffer_.size()-8);

		buffer_.push_back(checksum());
		
		return buffer_;
	}
};


/* all methods that use the control channel (sopas) */
class Control : public TCP_Session {
	Any_Session::SIG_ON_DATA on_data_;		///< signal handler for incoming data
	const std::string remote_device_ip_;
	bool stream_started_;

	void on_data(const char *data, const size_t size, Any_Session *writer)
	{
		RCLCPP_DEBUG(get_logger(), "got data for control");
	}
public:
	Control(boost::asio::io_service& io_service, const std::string &remote_device_ip) :
		TCP_Session(io_service, on_data_),
		remote_device_ip_(remote_device_ip),
		stream_started_(false)
	{
		on_data_.connect( boost::bind(&Control::on_data, this, _1, _2, _3) );
	}

    /* establish the control channel to the device */
    bool open() {
        RCLCPP_DEBUG(get_logger(), "Connecting to device...");

        if(!connect(remote_device_ip_, "2112")) {
            RCLCPP_ERROR(get_logger(), "Error on connecting to %s", remote_device_ip_.c_str());
            return false;
		}

        RCLCPP_DEBUG(get_logger(), "done.");
        return true;
    }

    /* Tells the device that there is a streaming channel by invoking a
     * method named GetBlobClientConfig.
     */
    bool initStream() {

        RCLCPP_DEBUG(get_logger(), "Initializing streaming...");
        CoLaFrame fr("sMN GetBlobClientConfig");
        RCLCPP_DEBUG(get_logger(), "Sending on sMN GetBlobClientConfig");
        write(fr.finish());
        RCLCPP_DEBUG(get_logger(), "done.");

        return true;
	}

    /* Start streaming the data by calling the "PLAYSTART" method on the
     * device and sending a "Blob request" afterwards.
     */
    bool startStream() {
		if(stream_started_) return true;

		CoLaFrame fr("sMN PLAYSTART");
        RCLCPP_DEBUG(get_logger(), "Sending on sopas sMN PLAYSTART");
        write(fr.finish());
        stream_started_ = true;

        return true;
	}


    /* Stops the data stream. */
    bool stopStream() {
        if(!stream_started_) return true;

		CoLaFrame fr("sMN PLAYSTOP");
        RCLCPP_DEBUG(get_logger(), "Sending on sopas sMN PLAYSTOP");
        write(fr.finish());
        stream_started_ = false;

        return true;
	}

private:
    static rclcpp::Logger get_logger() {
        return rclcpp::get_logger("Streaming");
    }
};



/* All methods that use the streaming channel. */
class Streaming : public TCP_Session {
public:

	struct SFrame {
		std::vector<char> buffer;

		typedef boost::shared_ptr<SFrame> Ptr;
	};

	typedef boost::signals2::signal<void (const boost::shared_ptr<Data> &)> SIG_ON_FRAME;

private:
    static rclcpp::Logger get_logger() {
        return rclcpp::get_logger("Streaming");
    }

	const std::string remote_device_ip_;
	Any_Session::SIG_ON_DATA on_data_;		///< signal handler for incoming data
	SIG_ON_FRAME on_frame_;
	bool debugOutput_;
	SFrame::Ptr cur_frame_;

public:

	Streaming(boost::asio::io_service& io_service, const std::string &remote_device_ip) :
		TCP_Session(io_service, on_data_),
		remote_device_ip_(remote_device_ip),
		debugOutput_(false)
	{
		on_data_.connect( boost::bind(&Streaming::on_data, this, _1, _2, _3) );
	}

	SIG_ON_FRAME &getSignal() {return on_frame_;}

	bool &debugFlag() 		{return debugOutput_;}
	bool debugFlag() const {return debugOutput_;}

    /* Opens the streaming channel. */
    bool openStream() {
        RCLCPP_DEBUG(get_logger(), "Opening streaming socket...");
        if(!connect(remote_device_ip_, "2113")) {
            RCLCPP_DEBUG(get_logger(), "Error on connecting to %s", remote_device_ip_.c_str());
            return false;
		}
        RCLCPP_DEBUG(get_logger(), "done.");

        // saying hello
        const char HEARTBEAT_MSG[] = "BlbReq";

        RCLCPP_DEBUG(get_logger(), "Sending BlbReq: %s", HEARTBEAT_MSG);
        write(std::string(HEARTBEAT_MSG, sizeof(HEARTBEAT_MSG)-1));

        return true;
	}

    /* Closes the streaming channel. */
    bool closeStream() {
        RCLCPP_DEBUG(get_logger(), "Closing streaming connection...");
        close();
        cur_frame_.reset();
        RCLCPP_DEBUG(get_logger(), "done.");

        return true;
	}

    /*
     *  Receives the raw data frame from the device via the streamingbchannel.
     */
	void on_data(const char *data, const size_t size, Any_Session *writer)
	{
        if(debugOutput_)
            RCLCPP_DEBUG(get_logger(), "Reading image from stream...");

        if(!data || size<0) {
            RCLCPP_DEBUG(get_logger(), "Socket not connected, terminating.");
            return;
		}

        if(cur_frame_) {
			cur_frame_->buffer.insert(cur_frame_->buffer.end(), data, data+size );

			bool goon=true;
			while(goon) {
				goon=false;

				if(Data::check_header(&cur_frame_->buffer[0], cur_frame_->buffer.size())) {
					boost::shared_ptr<Data> parser(new Data);
					const size_t actual_size = parser->actual_size(&cur_frame_->buffer[0], cur_frame_->buffer.size());
					if(parser->read(&cur_frame_->buffer[0], actual_size)) {
						on_frame_(parser);
						goon=true;
					}
					else {
						cur_frame_.reset();
						RCLCPP_ERROR(get_logger(), "failed to parse frame");
						break;
					}

					cur_frame_->buffer.erase(cur_frame_->buffer.begin(), cur_frame_->buffer.begin()+actual_size);
				}
			}
		}
		else {
			cur_frame_.reset(new SFrame());
			on_data(data, size, writer);
		}

		if(debugOutput_)
			RCLCPP_DEBUG(get_logger(), "done.");
	}
};

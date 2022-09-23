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

#pragma once

#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/thread/mutex.hpp>

#include "rclcpp/rclcpp.hpp"    //for ROS_ERROR, ROS_INFO, ...

//! abstract connection handler to process incoming data (independent of protocol)
class Any_Session {
public:
    /// incoming data with "data" of size "size" and handler to write back ("writer")
    typedef boost::signals2::signal<void(const char *data, const size_t size, Any_Session *writer)> SIG_ON_DATA;

    /// every session needs a data handler
    explicit Any_Session(SIG_ON_DATA &cb)
            : on_data_(cb) {
    }

    virtual ~Any_Session() = default;

    /// write a string to the network connection
    virtual void write(const std::string &buffer) = 0;

protected:

    SIG_ON_DATA &on_data_;    ///< signal handler for incoming data
};


//! implementation connection handler for TCP
class TCP_Session : public Any_Session {
public:

    TCP_Session(boost::asio::io_service &io_service, SIG_ON_DATA &cb)
            : Any_Session(cb), socket_(io_service) {
    }

    ~TCP_Session() override = default;

    /// getter for socket connection
    boost::asio::ip::tcp::socket &socket() {
        return socket_;
    }

    virtual bool connect(const std::string &path, const std::string &service) {
        boost::asio::ip::tcp::resolver resolver(socket_.get_io_service());
        boost::asio::ip::tcp::resolver::query query(path, service);
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::ip::tcp::resolver::iterator end;

        while (endpoint_iterator != end) {
            if (connect(*endpoint_iterator))
                return true;
        }

        return false;
    }

    virtual bool connect(const boost::asio::ip::tcp::resolver::endpoint_type &ep) {
        close();

        boost::system::error_code error = boost::asio::error::host_not_found;
        socket_.connect(ep, error);
        if (error) {
            ROS_ERROR("connecting failed");
            return false;//throw boost::system::system_error(error);
        }

        socket_.async_read_some(boost::asio::buffer(data_, max_length),
                                boost::bind(&TCP_Session::handle_read, this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
        return true;
    }

    virtual void close() {
        if (socket_.is_open()) socket_.close();
        boost::mutex::scoped_lock lock(mtx_);
    }

protected:

    /// write async. to TCP socket (implement abstract)
    virtual void write(const std::string &buffer) {
        boost::asio::async_write(socket_,
                                 boost::asio::buffer(buffer),
                                 boost::bind(&TCP_Session::handle_write, this,
                                             boost::asio::placeholders::error));
    }

    virtual void write(const std::vector<char> &buffer) {
        boost::asio::async_write(socket_,
                                 boost::asio::buffer(buffer),
                                 boost::bind(&TCP_Session::handle_write, this,
                                             boost::asio::placeholders::error));
    }

    /// write async. to TCP socket
    virtual void write(const boost::asio::mutable_buffers_1 &buffer) {
        boost::asio::async_write(socket_,
                                 buffer,
                                 boost::bind(&TCP_Session::handle_write, this,
                                             boost::asio::placeholders::error));
    }

private:

    /// called on incoming data --> decides if an error occured or calls handler
    void handle_read(const boost::system::error_code &error,
                     size_t bytes_transferred) {
        //boost::mutex::scoped_lock lock(mtx_);

        if (!error) {
            on_data_(data_, bytes_transferred, this);
        } else {
            ROS_ERROR("error while reading from socket");
            delete this;
        }

        if (socket_.is_open())
            socket_.async_read_some(boost::asio::buffer(data_, max_length),
                                    boost::bind(&TCP_Session::handle_read, this,
                                                boost::asio::placeholders::error,
                                                boost::asio::placeholders::bytes_transferred));
    }

    /// called if data is written (error handling)
    void handle_write(const boost::system::error_code &error) {
        //boost::mutex::scoped_lock lock(mtx_);

        if (error) {
            ROS_ERROR("error while writing to socket");
            delete this;
        }
    }

protected:
    boost::asio::ip::tcp::socket socket_;    ///< TCP socket
    enum {
        max_length = 4096
    };    ///< max length of incoming data (shoul be enough?)
    char data_[max_length]{};        ///< data buffer of max. length
    boost::mutex mtx_;            ///< lock operations
};

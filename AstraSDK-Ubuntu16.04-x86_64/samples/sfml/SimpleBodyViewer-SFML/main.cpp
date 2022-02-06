// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015-2017 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Be excellent to each other.
#include <SFML/Graphics.hpp>
#include <astra/astra.hpp>
#include <iostream>
#include <cstring>
#include <sstream>
#include "astra/streams/Body.hpp"

//include for BodyReaderPull
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

//include for SimipleStreamViewer
#include <astra_core/astra_core.hpp>
#include "LitDepthVisualizer.hpp"
#include <chrono>
#include <iomanip>

//include for communicate with robot
// #include "TimeClock.hpp"

#include <sys/types.h>
#include <sys/shm.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <functional>
#include <thread>
#include <vector>
#include <numeric>
using namespace std;
#include <math.h>

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
    #include <winsock2.h>
    #pragma comment(lib, "Ws2_32.lib")
#else
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <netdb.h>
    #include <unistd.h>
#endif

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;

struct CameraResponse {
    double  header;
    double  status;
    double Camera_Position_X;
    double Camera_Position_Y;
    double Camera_Position_Z;
    double Camera_orientation_X;
    double Camera_orientation_Y;
    double Camera_orientation_Z;
};

struct CameraData {
    double RightHand_X;
    double RightHand_Y;
    double RightHand_Z;
    double RightHandOritation_X;
    double RightHandOritation_Y;
    double RightHandOritation_Z;
};
typedef struct GetAverValueFromSensor {
	double right_hand_x;
	double right_hand_y;
	double right_hand_z;
	double right_hand_oritation_x;
	double right_hand_oritation_y;
	double right_hand_oritation_z;
}GetAverValue;



/*
code for SimpleBodyViewer below
*/
class sfLine : public sf::Drawable
{
public:
    sfLine(const sf::Vector2f& point1, const sf::Vector2f& point2, sf::Color color, float thickness)
        : color_(color)
    {
        const sf::Vector2f direction = point2 - point1;
        const sf::Vector2f unitDirection = direction / std::sqrt(direction.x*direction.x + direction.y*direction.y);
        const sf::Vector2f normal(-unitDirection.y, unitDirection.x);

        const sf::Vector2f offset = (thickness / 2.f) * normal;

        vertices_[0].position = point1 + offset;
        vertices_[1].position = point2 + offset;
        vertices_[2].position = point2 - offset;
        vertices_[3].position = point1 - offset;

        for (int i = 0; i<4; ++i)
            vertices_[i].color = color;
    }

    void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(vertices_, 4, sf::Quads, states);
    }

private:
    sf::Vertex vertices_[4];
    sf::Color color_;
};

class BodyVisualizer : public astra::FrameListener
{
public:
    BodyVisualizer()
    {
        font_.loadFromFile("Inconsolata.otf");
    }
    
    static sf::Color get_body_color(std::uint8_t bodyId)
    {
        if (bodyId == 0)
        {
            // Handle no body separately - transparent
            return sf::Color(0x00, 0x00, 0x00, 0x00);
        }
        // Case 0 below could mean bodyId == 25 or
        // above due to the "% 24".
        switch (bodyId % 24) {
        case 0:
            return sf::Color(0x00, 0x88, 0x00, 0xFF);
        case 1:
            return sf::Color(0x00, 0x00, 0xFF, 0xFF);
        case 2:
            return sf::Color(0x88, 0x00, 0x00, 0xFF);
        case 3:
            return sf::Color(0x00, 0xFF, 0x00, 0xFF);
        case 4:
            return sf::Color(0x00, 0x00, 0x88, 0xFF);
        case 5:
            return sf::Color(0xFF, 0x00, 0x00, 0xFF);

        case 6:
            return sf::Color(0xFF, 0x88, 0x00, 0xFF);
        case 7:
            return sf::Color(0xFF, 0x00, 0xFF, 0xFF);
        case 8:
            return sf::Color(0x88, 0x00, 0xFF, 0xFF);
        case 9:
            return sf::Color(0x00, 0xFF, 0xFF, 0xFF);
        case 10:
            return sf::Color(0x00, 0xFF, 0x88, 0xFF);
        case 11:
            return sf::Color(0xFF, 0xFF, 0x00, 0xFF);

        case 12:
            return sf::Color(0x00, 0x88, 0x88, 0xFF);
        case 13:
            return sf::Color(0x00, 0x88, 0xFF, 0xFF);
        case 14:
            return sf::Color(0x88, 0x88, 0x00, 0xFF);
        case 15:
            return sf::Color(0x88, 0xFF, 0x00, 0xFF);
        case 16:
            return sf::Color(0x88, 0x00, 0x88, 0xFF);
        case 17:
            return sf::Color(0xFF, 0x00, 0x88, 0xFF);

        case 18:
            return sf::Color(0xFF, 0x88, 0x88, 0xFF);
        case 19:
            return sf::Color(0xFF, 0x88, 0xFF, 0xFF);
        case 20:
            return sf::Color(0x88, 0x88, 0xFF, 0xFF);
        case 21:
            return sf::Color(0x88, 0xFF, 0xFF, 0xFF);
        case 22:
            return sf::Color(0x88, 0xFF, 0x88, 0xFF);
        case 23:
            return sf::Color(0xFF, 0xFF, 0x88, 0xFF);
        default:
            return sf::Color(0xAA, 0xAA, 0xAA, 0xFF);
        }
    }

    void init_depth_texture(int width, int height)
    {
        if (displayBuffer_ == nullptr || width != depthWidth_ || height != depthHeight_)
        {
            depthWidth_ = width;
            depthHeight_ = height;
            int byteLength = depthWidth_ * depthHeight_ * 4;

            displayBuffer_ = BufferPtr(new uint8_t[byteLength]);
            std::memset(displayBuffer_.get(), 0, byteLength);

            texture_.create(depthWidth_, depthHeight_);
            sprite_.setTexture(texture_, true);
            sprite_.setPosition(0, 0);
        }
    }

    void init_overlay_texture(int width, int height)
    {
        if (overlayBuffer_ == nullptr || width != overlayWidth_ || height != overlayHeight_)
        {
            overlayWidth_ = width;
            overlayHeight_ = height;
            int byteLength = overlayWidth_ * overlayHeight_ * 4;

            overlayBuffer_ = BufferPtr(new uint8_t[byteLength]);
            std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);

            overlayTexture_.create(overlayWidth_, overlayHeight_);
            overlaySprite_.setTexture(overlayTexture_, true);
            overlaySprite_.setPosition(0, 0);
        }
    }

    void check_fps()
    {
        double fpsFactor = 0.02;

        std::clock_t newTimepoint= std::clock();
        long double frameDuration = (newTimepoint - lastTimepoint_) / static_cast<long double>(CLOCKS_PER_SEC);

        frameDuration_ = frameDuration * fpsFactor + frameDuration_ * (1 - fpsFactor);
        lastTimepoint_ = newTimepoint;
        double fps = 1.0 / frameDuration_;

        // printf("FPS: %3.1f (%3.4Lf ms)\n", fps, frameDuration_ * 1000);
    }

    void processDepth(astra::Frame& frame)
    {
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

        if (!depthFrame.is_valid()) { return; }

        int width = depthFrame.width();
        int height = depthFrame.height();

        init_depth_texture(width, height);

        const int16_t* depthPtr = depthFrame.data();
        for(int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                int index = (x + y * width);
                int index4 = index * 4;

                int16_t depth = depthPtr[index];
                uint8_t value = depth % 255;

                displayBuffer_[index4] = value;
                displayBuffer_[index4 + 1] = value;
                displayBuffer_[index4 + 2] = value;
                displayBuffer_[index4 + 3] = 255;
            }
        }

        texture_.update(displayBuffer_.get());
    }

    void processBodies(astra::Frame& frame)
    {
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();

        jointPositions_.clear();
        circles_.clear();
        circleShadows_.clear();
        boneLines_.clear();
        boneShadows_.clear();

        if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
        {
            clear_overlay();
            return;
        }

        const float jointScale = bodyFrame.info().width() / 120.f;

        const auto& bodies = bodyFrame.bodies();

        for (auto& body : bodies)
        {
            // printf("Processing frame #%d body %d right hand: %u\n",
                // bodyFrame.frame_index(), body.id(), unsigned(body.hand_poses().right_hand()));
            for(auto& joint : body.joints())
            {
                jointPositions_.push_back(joint.depth_position());
                // printf("depth_position : ", joint.depth_position());
            }

            update_body(body, jointScale);
        }

        const auto& floor = bodyFrame.floor_info(); //floor
        if (floor.floor_detected())
        {
            const auto& p = floor.floor_plane();
            // std::cout << "Floor plane: ["
            //     << p.a() << ", " << p.b() << ", " << p.c() << ", " << p.d()
            //     << "]" << std::endl;

        }

        const auto& bodyMask = bodyFrame.body_mask();
        const auto& floorMask = floor.floor_mask();

        update_overlay(bodyMask, floorMask);
    }

    void update_body(astra::Body body,
                     const float jointScale)
    {
        const auto& joints = body.joints();

        if (joints.empty())
        {
            return;
        }

        for (const auto& joint : joints)
        {
            astra::JointType type = joint.type();
            const auto& pos = joint.depth_position();

            if (joint.status() == astra::JointStatus::NotTracked)
            {
                continue;
            }

            auto radius = jointRadius_ * jointScale; // pixels
            sf::Color circleShadowColor(0, 0, 0, 255);

             auto color = sf::Color(0x00, 0xFF, 0x00, 0xFF);

            if ((type == astra::JointType::LeftHand && astra::HandPose::Grip==body.hand_poses().left_hand()) ||
                (type == astra::JointType::RightHand &&  astra::HandPose::Grip==body.hand_poses().right_hand()))
            {
                radius *= 1.5f;
                circleShadowColor = sf::Color(255, 255, 255, 255);
                color = sf::Color(0x00, 0xAA, 0xFF, 0xFF);
            }

            const auto shadowRadius = radius + shadowRadius_ * jointScale;
            const auto radiusDelta = shadowRadius - radius;

            sf::CircleShape circle(radius);

            circle.setFillColor(sf::Color(color.r, color.g, color.b, 255));
            circle.setPosition(pos.x - radius, pos.y - radius);
            circles_.push_back(circle);

            sf::CircleShape shadow(shadowRadius);
            shadow.setFillColor(circleShadowColor);
            shadow.setPosition(circle.getPosition() - sf::Vector2f(radiusDelta, radiusDelta));
            circleShadows_.push_back(shadow);
        }

        update_bone(joints, jointScale, astra::JointType::Head, astra::JointType::Neck);
        update_bone(joints, jointScale, astra::JointType::Neck, astra::JointType::ShoulderSpine);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::LeftShoulder);
        update_bone(joints, jointScale, astra::JointType::LeftShoulder, astra::JointType::LeftElbow);
        update_bone(joints, jointScale, astra::JointType::LeftElbow, astra::JointType::LeftWrist);
        update_bone(joints, jointScale, astra::JointType::LeftWrist, astra::JointType::LeftHand);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::RightShoulder);
        update_bone(joints, jointScale, astra::JointType::RightShoulder, astra::JointType::RightElbow);
        update_bone(joints, jointScale, astra::JointType::RightElbow, astra::JointType::RightWrist);
        update_bone(joints, jointScale, astra::JointType::RightWrist, astra::JointType::RightHand);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::MidSpine);
        update_bone(joints, jointScale, astra::JointType::MidSpine, astra::JointType::BaseSpine);

        update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::LeftHip);
        update_bone(joints, jointScale, astra::JointType::LeftHip, astra::JointType::LeftKnee);
        update_bone(joints, jointScale, astra::JointType::LeftKnee, astra::JointType::LeftFoot);

        update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::RightHip);
        update_bone(joints, jointScale, astra::JointType::RightHip, astra::JointType::RightKnee);
        update_bone(joints, jointScale, astra::JointType::RightKnee, astra::JointType::RightFoot);
    }

    void update_bone(const astra::JointList& joints,
                     const float jointScale,astra::JointType j1,
                     astra::JointType j2)
    {
        const auto& joint1 = joints[int(j1)];
        const auto& joint2 = joints[int(j2)];

        if (joint1.status() == astra::JointStatus::NotTracked ||
            joint2.status() == astra::JointStatus::NotTracked)
        {
            //don't render bones between untracked joints
            return;
        }

        //actually depth position, not world position
        const auto& jp1 = joint1.depth_position();
        const auto& jp2 = joint2.depth_position();

        auto p1 = sf::Vector2f(jp1.x, jp1.y);
        auto p2 = sf::Vector2f(jp2.x, jp2.y);

        sf::Color color(255, 255, 255, 255);
        float thickness = lineThickness_ * jointScale;
        if (joint1.status() == astra::JointStatus::LowConfidence ||
            joint2.status() == astra::JointStatus::LowConfidence)
        {
            color = sf::Color(128, 128, 128, 255);
            thickness *= 0.5f;
        }

        boneLines_.push_back(sfLine(p1,
            p2,
            color,
            thickness));
        const float shadowLineThickness = thickness + shadowRadius_ * jointScale * 2.f;
        boneShadows_.push_back(sfLine(p1,
            p2,
            sf::Color(0, 0, 0, 255),
            shadowLineThickness));
    }

    void update_overlay(const astra::BodyMask& bodyMask,
                        const astra::FloorMask& floorMask)
    {
        const auto* bodyData = bodyMask.data();
        const auto* floorData = floorMask.data();
        const int width = bodyMask.width();
        const int height = bodyMask.height();

        init_overlay_texture(width, height);

        const int length = width * height;

        for (int i = 0; i < length; i++)
        {
            const auto bodyId = bodyData[i];
            const auto isFloor = floorData[i];

            sf::Color color(0x0, 0x0, 0x0, 0x0);

            if (bodyId != 0)
            {
                color = get_body_color(bodyId);
            }
            else if (isFloor != 0)
            {
                color = sf::Color(0x0, 0x0, 0xFF, 0x88);
            }

            const int rgbaOffset = i * 4;
            overlayBuffer_[rgbaOffset] = color.r;
            overlayBuffer_[rgbaOffset + 1] = color.g;
            overlayBuffer_[rgbaOffset + 2] = color.b;
            overlayBuffer_[rgbaOffset + 3] = color.a;
        }

        overlayTexture_.update(overlayBuffer_.get());
    }

    void clear_overlay()
    {
        int byteLength = overlayWidth_ * overlayHeight_ * 4;
        std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);

        overlayTexture_.update(overlayBuffer_.get());
    }

    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    {

        check_fps();
        if (isPaused_) { return; }
        
        processDepth(frame);
        processBodies(frame);
    }

    void draw_bodies(sf::RenderWindow& window)
    {
        const float scaleX = window.getView().getSize().x / overlayWidth_;
        const float scaleY = window.getView().getSize().y / overlayHeight_;

        sf::RenderStates states;
        sf::Transform transform;
        transform.scale(scaleX, scaleY);
        states.transform *= transform;

        for (const auto& bone : boneShadows_)
            window.draw(bone, states);

        for (const auto& c : circleShadows_)
            window.draw(c, states);

        for (const auto& bone : boneLines_)
            window.draw(bone, states);

        for (auto& c : circles_)
            window.draw(c, states);

    }

    void draw_text(sf::RenderWindow& window,
                   sf::Text& text,
                   sf::Color color,
                   const int x,
                   const int y) const
    {
        text.setColor(sf::Color::Black);
        text.setPosition(x + 5, y + 5);
        window.draw(text);

        text.setColor(color);
        text.setPosition(x, y);
        window.draw(text);
    }

    void draw_help_message(sf::RenderWindow& window) const
    {
        if (!isMouseOverlayEnabled_) {
            return;
        }

        std::stringstream str;
        str << "press h to toggle help message";

        if (isFullHelpEnabled_ && helpMessage_ != nullptr)
        {
            str << "\n" << helpMessage_;
        }

        const int characterSize = 30;
        sf::Text text(str.str(), font_);
        text.setCharacterSize(characterSize);
        text.setStyle(sf::Text::Bold);

        const float displayX = 0.f;
        const float displayY = 0;

        draw_text(window, text, sf::Color::White, displayX, displayY);
    }

    void draw_to(sf::RenderWindow& window)
    {
        if (displayBuffer_ != nullptr)
        {
            const float scaleX = window.getView().getSize().x / depthWidth_;
            const float scaleY = window.getView().getSize().y / depthHeight_;
            sprite_.setScale(scaleX, scaleY);

            window.draw(sprite_); // depth
        }

        if (overlayBuffer_ != nullptr)
        {
            const float scaleX = window.getView().getSize().x / overlayWidth_;
            const float scaleY = window.getView().getSize().y / overlayHeight_;
            overlaySprite_.setScale(scaleX, scaleY);
            window.draw(overlaySprite_); //bodymask and floormask
        }

        draw_bodies(window);
        draw_help_message(window);
    }

    void toggle_paused()
    {
        isPaused_ = !isPaused_;
    }

    bool is_paused() const
    {
        return isPaused_;
    }

    void toggle_overlay()
    {
        isMouseOverlayEnabled_ = !isMouseOverlayEnabled_;
    }

    bool overlay_enabled() const
    {
        return isMouseOverlayEnabled_;
    }

    void toggle_help()
    {
        isFullHelpEnabled_ = !isFullHelpEnabled_;
    }

    void set_help_message(const char* msg)
    {
        helpMessage_ = msg;
    }
private:
    long double frameDuration_{ 0 };
    std::clock_t lastTimepoint_ { 0 };
    sf::Texture texture_;
    sf::Sprite sprite_;
    sf::Font font_;

    using BufferPtr = std::unique_ptr < uint8_t[] >;
    BufferPtr displayBuffer_{ nullptr };

    std::vector<astra::Vector2f> jointPositions_;

    int depthWidth_{0};
    int depthHeight_{0};
    int overlayWidth_{0};
    int overlayHeight_{0};

    std::vector<sfLine> boneLines_;
    std::vector<sfLine> boneShadows_;
    std::vector<sf::CircleShape> circles_;
    std::vector<sf::CircleShape> circleShadows_;

    float lineThickness_{ 0.5f }; // pixels
    float jointRadius_{ 1.0f };   // pixels
    float shadowRadius_{ 0.5f };  // pixels

    BufferPtr overlayBuffer_{ nullptr };
    sf::Texture overlayTexture_;
    sf::Sprite overlaySprite_;

    bool isPaused_{false};
    bool isMouseOverlayEnabled_{true};
    bool isFullHelpEnabled_{false};
    const char* helpMessage_{nullptr};
};

astra::DepthStream configure_depth(astra::StreamReader& reader)
{
    auto depthStream = reader.stream<astra::DepthStream>();

    //We don't have to set the mode to start the stream, but if you want to here is how:
    astra::ImageStreamMode depthMode;

    depthMode.set_width(640);
    depthMode.set_height(480);
    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
    depthMode.set_fps(30);

    depthStream.set_mode(depthMode);

    return depthStream;
}

/*
code below is for simple stream viewer
*/
enum ColorMode
    {
        MODE_COLOR,
        MODE_IR_16,
        MODE_IR_RGB,
    };
class MultiFrameListener : public astra::FrameListener
{
public:
    using BufferPtr = std::unique_ptr<uint8_t[]>;

    struct StreamView
    {
        sf::Sprite sprite_;
        sf::Texture texture_;
        BufferPtr buffer_;
        int width_{0};
        int height_{0};
    };

    MultiFrameListener()
    {
        prev_ = ClockType::now();
    }

    void init_texture(int width, int height, StreamView& view)
    {
        if (view.buffer_ == nullptr || width != view.width_ || height != view.height_)
        {
            view.width_ = width;
            view.height_ = height;

            // texture is RGBA
            const int byteLength = width * height * 4;

            view.buffer_ = BufferPtr(new uint8_t[byteLength]);

            clear_view(view);

            view.texture_.create(width, height);
            view.sprite_.setTexture(view.texture_, true);
            view.sprite_.setPosition(0, 0);
        }
    }

    void clear_view(StreamView& view)
    {
        const int byteLength = view.width_ * view.height_ * 4;
        std::fill(&view.buffer_[0], &view.buffer_[0] + byteLength, 0);
    }

    void check_fps()
    {
        const float frameWeight = .2f;

        const ClockType::time_point now = ClockType::now();
        const float elapsedMillis = std::chrono::duration_cast<DurationType>(now - prev_).count();

        elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
        prev_ = now;

        const float fps = 1000.f / elapsedMillis;

        const auto precision = std::cout.precision();

        std::cout << std::fixed
                  << std::setprecision(1)
                  << fps << " fps ("
                  << std::setprecision(1)
                  << elapsedMillis_ << " ms)"
                  << std::setprecision(precision)
                  << std::endl;
    }

    void update_depth(astra::Frame& frame)
    {
        const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

        if (!pointFrame.is_valid())
        {
            clear_view(depthView_);
            depthView_.texture_.update(&depthView_.buffer_[0]);
            return;
        }

        const int depthWidth = pointFrame.width();
        const int depthHeight = pointFrame.height();

        init_texture(depthWidth, depthHeight, depthView_);

        if (isPaused_) { return; }

        visualizer_.update(pointFrame);

        astra::RgbPixel* vizBuffer = visualizer_.get_output();
        uint8_t* buffer = &depthView_.buffer_[0];
        for (int i = 0; i < depthWidth * depthHeight; i++)
        {
            const int rgbaOffset = i * 4;
            buffer[rgbaOffset] = vizBuffer[i].r;
            buffer[rgbaOffset + 1] = vizBuffer[i].g;
            buffer[rgbaOffset + 2] = vizBuffer[i].b;
            buffer[rgbaOffset + 3] = 255;
        }

        depthView_.texture_.update(&depthView_.buffer_[0]);
    }

    void update_color(astra::Frame& frame)
    {
        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

        if (!colorFrame.is_valid())
        {
            clear_view(colorView_);
            colorView_.texture_.update(&colorView_.buffer_[0]);
            return;
        }

        const int colorWidth = colorFrame.width();
        const int colorHeight = colorFrame.height();

        init_texture(colorWidth, colorHeight, colorView_);

        if (isPaused_) { return; }

        const astra::RgbPixel* color = colorFrame.data();
        uint8_t* buffer = &colorView_.buffer_[0];

        for(int i = 0; i < colorWidth * colorHeight; i++)
        {
            const int rgbaOffset = i * 4;
            buffer[rgbaOffset] = color[i].r;
            buffer[rgbaOffset + 1] = color[i].g;
            buffer[rgbaOffset + 2] = color[i].b;
            buffer[rgbaOffset + 3] = 255;
        }

        colorView_.texture_.update(&colorView_.buffer_[0]);
    }

    void update_ir_16(astra::Frame& frame)
    {
        const astra::InfraredFrame16 irFrame = frame.get<astra::InfraredFrame16>();

        if (!irFrame.is_valid())
        {
            clear_view(colorView_);
            colorView_.texture_.update(&colorView_.buffer_[0]);
            return;
        }

        const int irWidth = irFrame.width();
        const int irHeight = irFrame.height();

        init_texture(irWidth, irHeight, colorView_);

        if (isPaused_) { return; }

        const uint16_t* ir_values = irFrame.data();
        uint8_t* buffer = &colorView_.buffer_[0];
        for (int i = 0; i < irWidth * irHeight; i++)
        {
            const int rgbaOffset = i * 4;
            const uint16_t value = ir_values[i];
            const uint8_t red = static_cast<uint8_t>(value >> 2);
            const uint8_t blue = 0x66 - red / 2;
            buffer[rgbaOffset] = red;
            buffer[rgbaOffset + 1] = 0;
            buffer[rgbaOffset + 2] = blue;
            buffer[rgbaOffset + 3] = 255;
        }

        colorView_.texture_.update(&colorView_.buffer_[0]);
    }

    void update_ir_rgb(astra::Frame& frame)
    {
        const astra::InfraredFrameRgb irFrame = frame.get<astra::InfraredFrameRgb>();

        if (!irFrame.is_valid())
        {
            clear_view(colorView_);
            colorView_.texture_.update(&colorView_.buffer_[0]);
            return;
        }

        int irWidth = irFrame.width();
        int irHeight = irFrame.height();

        init_texture(irWidth, irHeight, colorView_);

        if (isPaused_) { return; }

        const astra::RgbPixel* irRGB = irFrame.data();
        uint8_t* buffer = &colorView_.buffer_[0];
        for (int i = 0; i < irWidth * irHeight; i++)
        {
            const int rgbaOffset = i * 4;
            buffer[rgbaOffset] = irRGB[i].r;
            buffer[rgbaOffset + 1] = irRGB[i].g;
            buffer[rgbaOffset + 2] = irRGB[i].b;
            buffer[rgbaOffset + 3] = 255;
        }

        colorView_.texture_.update(&colorView_.buffer_[0]);
    }

    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    {
        update_depth(frame);

        switch (colorMode_)
        {
        case MODE_COLOR:
            update_color(frame);
            break;
        case MODE_IR_16:
            update_ir_16(frame);
            break;
        case MODE_IR_RGB:
            update_ir_rgb(frame);
            break;
        }

        check_fps();
    }

    void draw_to(sf::RenderWindow& window, sf::Vector2f origin, sf::Vector2f size)
    {
        const int viewSize = (int)(size.x / 2.0f);
        const sf::Vector2f windowSize = window.getView().getSize();

        if (depthView_.buffer_ != nullptr)
        {
            const float depthScale = viewSize / (float)depthView_.width_;
            const int horzCenter = origin.y * windowSize.y;

            depthView_.sprite_.setScale(depthScale, depthScale);
            depthView_.sprite_.setPosition(origin.x * windowSize.x, horzCenter);
            window.draw(depthView_.sprite_);
        }

        if (colorView_.buffer_ != nullptr)
        {
            const float colorScale = viewSize / (float)colorView_.width_;
            const int horzCenter = origin.y * windowSize.y;

            colorView_.sprite_.setScale(colorScale, colorScale);

            if (overlayDepth_)
            {
                colorView_.sprite_.setPosition(origin.x * windowSize.x, horzCenter);
                colorView_.sprite_.setColor(sf::Color(255, 255, 255, 128));
            }
            else
            {
                colorView_.sprite_.setPosition(origin.x * windowSize.x + viewSize, horzCenter);
                colorView_.sprite_.setColor(sf::Color(255, 255, 255, 255));
            }

            window.draw(colorView_.sprite_);
        }
    }

    void toggle_depth_overlay()
    {
        overlayDepth_ = !overlayDepth_;
    }

    bool get_overlay_depth() const
    {
        return overlayDepth_;
    }

    void toggle_paused()
    {
        isPaused_ = !isPaused_;
    }

    bool is_paused() const
    {
        return isPaused_;
    }

    ColorMode get_mode() const { return colorMode_; }
    void set_mode(ColorMode mode) { colorMode_ = mode; }

private:
    samples::common::LitDepthVisualizer visualizer_;

    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;

    ClockType::time_point prev_;
    float elapsedMillis_{.0f};

    StreamView depthView_;
    StreamView colorView_;
    ColorMode colorMode_;
    bool overlayDepth_{ false };
    bool isPaused_{ false };
};

astra::InfraredStream configure_ir(astra::StreamReader& reader, bool useRGB)
{
    auto irStream = reader.stream<astra::InfraredStream>();

    auto oldMode = irStream.mode();

    astra::ImageStreamMode irMode;
    irMode.set_width(640);
    irMode.set_height(480);

    if (useRGB)
    {
        irMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
    }
    else
    {
        irMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_GRAY16);
    }

    irMode.set_fps(30);
    irStream.set_mode(irMode);

    auto newMode = irStream.mode();
    printf("Changed IR mode: %dx%d @ %d -> %dx%d @ %d\n",
        oldMode.width(), oldMode.height(), oldMode.fps(),
        newMode.width(), newMode.height(), newMode.fps());

    return irStream;
}

astra::ColorStream configure_color(astra::StreamReader& reader)
{
    auto colorStream = reader.stream<astra::ColorStream>();

    auto oldMode = colorStream.mode();

    astra::ImageStreamMode colorMode;
    colorMode.set_width(640);
    colorMode.set_height(480);
    colorMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
    colorMode.set_fps(30);

    colorStream.set_mode(colorMode);

    auto newMode = colorStream.mode();
    printf("Changed color mode: %dx%d @ %d -> %dx%d @ %d\n",
        oldMode.width(), oldMode.height(), oldMode.fps(),
        newMode.width(), newMode.height(), newMode.fps());

    return colorStream;
}
/* **** gap line **** */



/*
code below is for BodyReaderPull 
*/
void output_floor(astra_bodyframe_t bodyFrame)
{
    astra_floor_info_t floorInfo;

    astra_status_t rc = astra_bodyframe_floor_info(bodyFrame, &floorInfo);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_floor_info()\n", rc);
        return;
    }

    const astra_bool_t floorDetected = floorInfo.floorDetected;
    const astra_plane_t* floorPlane = &floorInfo.floorPlane;
    const astra_floormask_t* floorMask = &floorInfo.floorMask;

    if (floorDetected != ASTRA_FALSE)
    {
        printf("Floor plane: [%f, %f, %f, %f]\n",
               floorPlane->a,
               floorPlane->b,
               floorPlane->c,
               floorPlane->d);

        const int32_t bottomCenterIndex = floorMask->width / 2 + floorMask->width * (floorMask->height - 1);
        printf("Floor mask: width: %d height: %d bottom center value: %d\n",
            floorMask->width,
            floorMask->height,
            floorMask->data[bottomCenterIndex]);
    }
}

void output_body_mask(astra_bodyframe_t bodyFrame)
{
    astra_bodymask_t bodyMask;

    const astra_status_t rc = astra_bodyframe_bodymask(bodyFrame, &bodyMask);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_bodymask()\n", rc);
        return;
    }

    const int32_t centerIndex = bodyMask.width / 2 + bodyMask.width * bodyMask.height / 2;
    printf("Body mask: width: %d height: %d center value: %d\n",
        bodyMask.width,
        bodyMask.height,
        bodyMask.data[centerIndex]);
}

void output_bodyframe_info(astra_bodyframe_t bodyFrame)
{
    astra_bodyframe_info_t info;

    const astra_status_t rc = astra_bodyframe_info(bodyFrame, &info);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_info()\n", rc);
        return;
    }

    // width and height of floor mask, body mask, and the size of depth image
    // that joint depth position is relative to.
    const int32_t width = info.width;
    const int32_t height = info.height;

    printf("BodyFrame info: Width: %d Height: %d\n",
        width,
        height);
}

GetAverValue getAverValue;
void output_joint(const int32_t bodyId, const astra_joint_t* joint)
{
    // jointType is one of ASTRA_JOINT_* which exists for each joint type
    const astra_joint_type_t jointType = joint->type;

    // jointStatus is one of:
    // ASTRA_JOINT_STATUS_NOT_TRACKED = 0,
    // ASTRA_JOINT_STATUS_LOW_CONFIDENCE = 1,
    // ASTRA_JOINT_STATUS_TRACKED = 2,
    const astra_joint_status_t jointStatus = joint->status;

    const astra_vector3f_t* worldPos = &joint->worldPosition;

    // depthPosition is in pixels from 0 to width and 0 to height
    // where width and height are member of astra_bodyframe_info_t
    // which is obtained from astra_bodyframe_info().
    const astra_vector2f_t* depthPos = &joint->depthPosition;

    getAverValue.right_hand_x = worldPos->x;
    getAverValue.right_hand_y = worldPos->z;//note: it was showed that physical meaning of z and y are opposite,so here 
    getAverValue.right_hand_z = worldPos->y;
    printf("Body %u Joint %d status %d @ Right_hand_world_position (%.1f, %.1f, %.1f) depth (%.1f, %.1f)\n",
           bodyId,
           jointType,
           jointStatus,
           worldPos->x,
           worldPos->z,
           worldPos->y,
           depthPos->x,
           depthPos->y);

    // orientation is a 3x3 rotation matrix where the column vectors also
    // represent the orthogonal basis vectors for the x, y, and z axes.
    const astra_matrix3x3_t* orientation = &joint->orientation;
    const astra_vector3f_t* xAxis = &orientation->xAxis; // same as orientation->m00, m10, m20
    const astra_vector3f_t* yAxis = &orientation->yAxis; // same as orientation->m01, m11, m21
    const astra_vector3f_t* zAxis = &orientation->zAxis; // same as orientation->m02, m12, m22

    printf("Right Hand orientation x: [%f %f %f]\n", xAxis->x, xAxis->y, xAxis->z);
    printf("Right Hand orientation y: [%f %f %f]\n", yAxis->x, yAxis->y, yAxis->z);
    printf("Right Hand orientation z: [%f %f %f]\n", zAxis->x, zAxis->y, zAxis->z);
}

void output_hand_poses(const astra_body_t* body)
{
    const astra_handpose_info_t* handPoses = &body->handPoses;

    // astra_handpose_t is one of:
    // ASTRA_HANDPOSE_UNKNOWN = 0
    // ASTRA_HANDPOSE_GRIP = 1
    const astra_handpose_t leftHandPose = handPoses->leftHand;
    const astra_handpose_t rightHandPose = handPoses->rightHand;

    // printf("Body %d Left hand pose: %d Right hand pose: %d\n",
    //     body->id,
    //     leftHandPose,
    //     rightHandPose);
}

void output_bodies(astra_bodyframe_t bodyFrame)
{
    int i;
    astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_body_list()\n", rc);
        return;
    }

    for(i = 0; i < bodyList.count; ++i)
    {
        astra_body_t* body = &bodyList.bodies[i];

        // Pixels in the body mask with the same value as bodyId are
        // from the same body.
        astra_body_id_t bodyId = body->id;

        // bodyStatus is one of:
        // ASTRA_BODY_STATUS_NOT_TRACKING = 0,
        // ASTRA_BODY_STATUS_LOST = 1,
        // ASTRA_BODY_STATUS_TRACKING_STARTED = 2,
        // ASTRA_BODY_STATUS_TRACKING = 3,
        astra_body_status_t bodyStatus = body->status;

        if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED)
        {
            printf("Body Id: %d Status: Tracking started\n", bodyId);
        }
        if (bodyStatus == ASTRA_BODY_STATUS_TRACKING)
        {
            printf("Body Id: %d Status: Tracking\n", bodyId);
        }

        if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED ||
            bodyStatus == ASTRA_BODY_STATUS_TRACKING)
        {
            const astra_vector3f_t* centerOfMass = &body->centerOfMass;

            const astra_body_tracking_feature_flags_t features = body->features;
            const bool jointTrackingEnabled       = (features & ASTRA_BODY_TRACKING_JOINTS)     == ASTRA_BODY_TRACKING_JOINTS;
            const bool handPoseRecognitionEnabled = (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

            // printf("Body %d CenterOfMass (%f, %f, %f) Joint Tracking Enabled: %s Hand Pose Recognition Enabled: %s\n",
            //     bodyId,
            //     centerOfMass->x, centerOfMass->y, centerOfMass->z,
            //     jointTrackingEnabled       ? "True" : "False",
            //     handPoseRecognitionEnabled ? "True" : "False");

            const astra_joint_t* joint = &body->joints[ASTRA_JOINT_RIGHT_HAND];

            output_joint(bodyId, joint);

            output_hand_poses(body);
        }
        else if (bodyStatus == ASTRA_BODY_STATUS_LOST)
        {
            printf("Body %u Status: Tracking lost.\n", bodyId);
        }
        else // bodyStatus == ASTRA_BODY_STATUS_NOT_TRACKING
        {
            printf("Body Id: %d Status: Not Tracking\n", bodyId);
        }
    }
}

void output_bodyframe(astra_bodyframe_t bodyFrame)
{
    // output_floor(bodyFrame);

    // output_body_mask(bodyFrame);

    // output_bodyframe_info(bodyFrame);

    output_bodies(bodyFrame);
}



/**
 * @brief Camera传感器设备类
 * 用于对Camera传感器设备的控制
 */
class CameraSensor :public MultiFrameListener, public BodyVisualizer
{
private:
    /* data */
    const char *ipAddress = "192.168.1.1";
    const uint16 port = 49151;
    bool isPrint = false;
    int duration = 10;
    bool canStartThreading = true;
    int shm_id;
    int shm_keyValue;
    struct CameraData *shm_cameraData=NULL;

    struct CameraResponse cameraResponse;
    // CameraData ftAbsValue = {0, 0, 0, 0, 0, 0};
    CameraData cameraAverValue = {0, 0, 0, 0, 0, 0};
    CameraData SensorCoefficient  = {1, 1, 1, 1, 1, 1};
    CameraData MinThreshold = {0.2, 0.4, 0.8, 0.02, 0.02, 0.02};

    



public:
    CameraSensor();
    CameraSensor(int keyValue);
    ~CameraSensor();

    // MultiFrameListener listener_stream;
    // BodyVisualizer listener;

    /* 大小端字节调整 */
    int16 swap_int16(int16 val);    
    void SwapCameraResponseBytes();

    /* 连接Camera传感器设备 */
    int Open();

    /* 获取标定信息 */
    int GetCalibrationInfo();
    /* 打印标定信息 */
    void ShowCalibrationInfo();


    /* 读取ft数据 */
    int ReadCamera();
    void processCameraData();

    /*读取时通过平均值读取*/
    void ReadCamerabyMean();

    /* 打印ft数据 */
    void ShowResponse();

    void ftCycle();
    void startThread();

    /* 获得Camera传感器的初始平均值 */
    void getMeanValue();

    /* 获得Camera传感器的实时绝对值 */
    void getAbsoluteValue();

    /* 0附近邻域值舍入 */
    void roundThrehold(CameraData &ftValue);
    

    void setPrintFlag(bool flag, int fps=100);

    void getWindow();

};
CameraSensor::CameraSensor()
{
    this->shm_keyValue = 1002;
 


    cout<<"构造函数"<<endl;

}
CameraSensor::CameraSensor(int keyValue)
{
    this->shm_keyValue = keyValue;
}

CameraSensor::~CameraSensor()
{
}

void CameraSensor::getWindow()
{

}
int CameraSensor::Open()
{
    
    void *shm = NULL;
    this->shm_id = shmget((key_t)this->shm_keyValue, 0, 0);
    if(this->shm_id != -1)
    {
        shmctl(shm_id, IPC_RMID, 0);
    }
    this->shm_id = shmget((key_t)this->shm_keyValue, sizeof(struct CameraData), 0666|IPC_CREAT);
    if(this->shm_id == -1)
    {
        fprintf(stderr, "shm of ft get failed\n");
        exit(EXIT_FAILURE);
    }
    shm = shmat(shm_id, (void*)0, 0);
    if(shm == (void*)-1) 
    {
        fprintf(stderr, "shmat of ft get failed\n");
        exit(EXIT_FAILURE);
    }
    this->shm_cameraData = (struct CameraData *)shm;
    
    return 0;
}
/*
code for BodyReaderPoll below
*/
/* 读取camera数据 */
int CameraSensor::ReadCamera()
{
    printf("BodyReaderPoll begin \n");
    astra_streamsetconnection_t sensor_body_reader_poll;

    astra_streamset_open("device/default", &sensor_body_reader_poll);

    astra_reader_t reader_body_reader_poll;
    astra_reader_create(sensor_body_reader_poll, &reader_body_reader_poll);

    astra_bodystream_t bodyStream_body_reader_poll;
    astra_reader_get_bodystream(reader_body_reader_poll, &bodyStream_body_reader_poll);

    astra_stream_start(bodyStream_body_reader_poll);

    /*
    code for BodyReaderPoll below
    */
    astra_update(); //如果没有这个函数，那么在没有update的情况下，相机检测不到新的success状态，则无法进入到if中。
    astra_reader_frame_t frame;
    astra_status_t rc = astra_reader_open_frame(reader_body_reader_poll, 0, &frame);
    cout<<rc<<endl;
    if (rc == ASTRA_STATUS_SUCCESS)
    {
        astra_bodyframe_t bodyFrame;
        astra_frame_get_bodyframe(frame, &bodyFrame);
        astra_frame_index_t frameIndex;
        astra_bodyframe_get_frameindex(bodyFrame, &frameIndex);
        printf("Frame index: %d\n", frameIndex);
        output_bodyframe(bodyFrame);
        printf("----------------------------\n");
        astra_reader_close_frame(&frame);
    }
    /*gap line*/




    return 0;
}


void CameraSensor::ReadCamerabyMean()
{
    vector<double> Fx(5);
    vector<double> Fy(5);
    vector<double> Fz(5);
    vector<double> Tx(5);
    vector<double> Ty(5);
    vector<double> Tz(5);
    for (size_t i = 0; i < 5; i++)
    {
        ReadCamera();
        Fx[i]= getAverValue.right_hand_x;
	    Fy[i]= getAverValue.right_hand_y;
	    Fz[i]= getAverValue.right_hand_z;
	    Tx[i]= getAverValue.right_hand_oritation_x;
	    Ty[i]= getAverValue.right_hand_oritation_y;
        Tz[i]= getAverValue.right_hand_oritation_z;	 
    }
    
    cameraResponse.Camera_Position_X = (accumulate(Fx.begin(), Fx.end(), 0.0))/Fx.size();
    cameraResponse.Camera_Position_Y = (accumulate(Fy.begin(), Fy.end(), 0.0))/Fy.size();
    cameraResponse.Camera_Position_Z = (accumulate(Fz.begin(), Fz.end(), 0.0))/Fz.size();
    cameraResponse.Camera_orientation_X = (accumulate(Tx.begin(), Tx.end(), 0.0))/Tx.size();
    cameraResponse.Camera_orientation_Y = (accumulate(Ty.begin(), Ty.end(), 0.0))/Ty.size();
    cameraResponse.Camera_orientation_Z = (accumulate(Tz.begin(), Tz.end(), 0.0))/Tz.size();

    this->shm_cameraData->RightHand_X         = cameraResponse.Camera_Position_X;
    this->shm_cameraData->RightHand_Y         = cameraResponse.Camera_Position_Y;
    this->shm_cameraData->RightHand_Z         = cameraResponse.Camera_Position_Z;
    this->shm_cameraData->RightHandOritation_X= cameraResponse.Camera_orientation_X;
    this->shm_cameraData->RightHandOritation_Y= cameraResponse.Camera_orientation_Y;
    this->shm_cameraData->RightHandOritation_Z= cameraResponse.Camera_orientation_Z;

    printf("mean value: RightHand_X:%.4f  RightHand_Y:%.4f RightHand_Z: %.4f  RightHandOritation_X %.4f\r\n", 
    this->shm_cameraData->RightHand_X,
    this->shm_cameraData->RightHand_Y,
    this->shm_cameraData->RightHand_Z,
    this->shm_cameraData->RightHandOritation_X );

}





int main(int argc, char** argv)
{
    astra::initialize();
    CameraSensor CS;
    CS.Open();
    // CS.getWindow();
    
    if (argc == 2)
    {
        FILE *fp = fopen(argv[1], "rb");
        char licenseString[1024] = { 0 };
        fread(licenseString, 1, 1024, fp);
        orbbec_body_tracking_set_license(licenseString);

        fclose(fp);
    }
    else
    {
        const char* licenseString = "<INSERT LICENSE KEY HERE>";
        orbbec_body_tracking_set_license(licenseString);
    }

#ifdef _WIN32
    auto fullscreenStyle = sf::Style::None;
#else
    auto fullscreenStyle = sf::Style::Fullscreen;
#endif

    const sf::VideoMode fullScreenMode = sf::VideoMode::getFullscreenModes()[0];
    const sf::VideoMode windowedMode(1280, 960);
    //set up two windows
    sf::RenderWindow window(sf::VideoMode(1280, 960), "Simple Body Viewer");
    // sf::RenderWindow *pWindow;
    // pWindow = &window;
    sf::RenderWindow window1(windowedMode, "Stream Viewer");

    MultiFrameListener listener_stream;
    BodyVisualizer listener;

    //test: 先加入了另外两个窗口的启动，最下面一部分时BodyReaderPoll, 注意 gap line
    /*
    code below is for simple stream viewer
    */
    astra::StreamSet streamSet;
    astra::StreamReader reader_stream = streamSet.create_reader();

    reader_stream.stream<astra::PointStream>().start();

    auto colorStream = configure_color(reader_stream);
    colorStream.start();

    auto irStream = configure_ir(reader_stream, false);

    listener_stream.set_mode(MODE_COLOR);

    reader_stream.add_listener(listener_stream);
    /* **** gap line **** */

    /*
    code below is for simple body viewer
    */
    bool isFullScreen = false;

    astra::StreamSet sensor;
    astra::StreamReader reader = sensor.create_reader();


    auto depthStream = configure_depth(reader);
    depthStream.start();

    auto bodyStream = reader.stream<astra::BodyStream>();

    const char* helpMessage = 
        "keyboard shortcut:\n"
        "D      use 640x400 depth resolution\n"
        "F      toggle between fullscreen and windowed mode\n"
        "H      show/hide this message\n"
        "M      enable/disable depth mirroring\n"
        "P      enable/disable drawing texture\n"
        "S      toggle SkeletonProfile(Full or Basic)\n"
        "T      toggle BodyTrackingFeatureFlags(HandPoses, Segmentation or Joints\n"
        "SPACE  show/hide all text\n"
        "Esc    exit";
    listener.set_help_message(helpMessage);
    bodyStream.start();
    reader.add_listener(listener);

    astra::SkeletonProfile profile = bodyStream.get_skeleton_profile();

    // HandPoses includes Joints and Segmentation
    astra::BodyTrackingFeatureFlags features = astra::BodyTrackingFeatureFlags::HandPoses;
    /* **** gap line **** */





    while (window.isOpen())
    // while (1)
    {
        // astra_update();
        CS.ReadCamerabyMean();//namely BodyReaderPoll

        /**
         *  TODO 
         * 将以下window的draw_to，display 放到 ReadCamerabyMean() 的循环中去，提高video窗口的刷新频率。
         * */
        // clear the window with black color
        window.clear(sf::Color::Black);
        listener.draw_to(window);
        window.display();

        window1.clear(sf::Color::Black);
        listener_stream.draw_to(window1, sf::Vector2f(0.f, 0.f), sf::Vector2f(window1.getSize().x, window1.getSize().y));
        window1.display();

 

        sf::Event event;
        while (window.pollEvent(event) | window1.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;
            case sf::Event::KeyPressed:
            {
                if (event.key.code == sf::Keyboard::C && event.key.control)
                {
                    window.close();
                }
                switch (event.key.code)
                {
                case sf::Keyboard::D:
                {
                    auto oldMode = depthStream.mode();
                    astra::ImageStreamMode depthMode;

                    depthMode.set_width(640);
                    depthMode.set_height(400);
                    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
                    depthMode.set_fps(30);

                    depthStream.set_mode(depthMode);
                    auto newMode = depthStream.mode();
                    printf("Changed depth mode: %dx%d @ %d -> %dx%d @ %d\n",
                           oldMode.width(), oldMode.height(), oldMode.fps(),
                           newMode.width(), newMode.height(), newMode.fps());
                    break;
                }
                case sf::Keyboard::Escape:
                    window.close();
                    break;
                case sf::Keyboard::F:
                    if (isFullScreen)
                    {
                        window.create(windowedMode, "Simple Body Viewer", sf::Style::Default);
                    }
                    else
                    {
                        window.create(fullScreenMode, "Simple Body Viewer", fullscreenStyle);
                    }
                    isFullScreen = !isFullScreen;
                    break;
                case sf::Keyboard::H:
                    listener.toggle_help();
                    break;
                case sf::Keyboard::M:
                    depthStream.enable_mirroring(!depthStream.mirroring_enabled());
                    break;
                case sf::Keyboard::P:
                    listener.toggle_paused();
                    break;
                case sf::Keyboard::S:
                    if (profile == astra::SkeletonProfile::Full)
                    {
                        profile = astra::SkeletonProfile::Basic;
                        printf("Skeleton Profile: basic\n");
                    }
                    else
                    {
                        profile = astra::SkeletonProfile::Full;
                        printf("Skeleton Profile: full\n");
                    }
                    bodyStream.set_skeleton_profile(profile);
                    break;
                case sf::Keyboard::T:
                    if (features == astra::BodyTrackingFeatureFlags::Segmentation)
                    {
                        // Joints includes Segmentation
                        features = astra::BodyTrackingFeatureFlags::Joints;
                        printf("Default Body Features: Seg+Body\n");
                    }
                    else if (features == astra::BodyTrackingFeatureFlags::Joints)
                    {
                        // HandPoses includes Joints and Segmentation
                        features = astra::BodyTrackingFeatureFlags::HandPoses;
                        printf("Default Body Features: Seg+Body+Hand\n");
                    }
                    else
                    {
                        // HandPoses includes Joints and Segmentation
                        features = astra::BodyTrackingFeatureFlags::Segmentation;
                        printf("Default Body Features: Seg\n");
                    }
                    bodyStream.set_default_body_features(features);
                    break;
                case sf::Keyboard::Space:
                    listener.toggle_overlay();
                    break;
                default:
                    break;
                }
                break;
            }
            default:
                break;
            }
        }


    }

    astra::terminate();

    return 0;
}

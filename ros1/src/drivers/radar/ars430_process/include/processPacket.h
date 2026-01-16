#ifndef PROCESS_PACKET_H
#define PROCESS_PACKET_H

#pragma once

/* File: processPacket.h
 *
 * Groups incoming packets into buffer based on timestamp & type
 * Performs simple filtering & thresholding on simeple packet contents
 */

/* General Use Includes */
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>

#include <ars430_process/RadarDetection.h>
#include <ars430_process/RadarPacket.h>

#include <ars430_process/SensorStatus.h>

#include "common.h"
#include "data_struct.h"
#include "info_convert_node.h"

// Logging Verbosity Levels
enum LoggingLevel {LOG_NONE, LOG_DEBUG, LOG_WARNING, LOG_ERROR};

//Filter Thresholds
#define SNR_THRESHOLD            3 //dBr
#define VELOCITY_LOWER_THRESHOLD 0.00 // m/s
#define RCS_THRESHOLD           -70 //(dBm)^2
#define DISTANCE_MAX_THRESHOLD   70 //m
#define DISTANCE_MIN_THRESHOLD   0.25 //m
#define AZI_ANGLE_0_THRESHOLD    -1.44862 //radians, 83 degrees Azi Angle 0 is the angle from the left center
#define AZI_ANGLE_1_THRESHOLD    1.44862 //Azi Angle 1 is the angle from the right center, positive according to Conti

class InfoConverNode;

static PacketGroup_t EmptyPackets; //Local static only, Cleanup struct

class PacketProcessor {
    private:
        InfoConverNode*          Publisher;
        pthread_mutex_t          Mutex; //Mutex for synchronization
        PacketGroup_t            PacketsBuffer[2]; //Double buffer
        uint8_t                  curNearIdx, curFarIdx; //Double buffer indexes
        uint32_t                 curNearTimeStamp, curFarTimeStamp; //Time Stamps
        bool                     publish;

        uint8_t clearPackets(uint8_t idx); //Don't expose these
        uint8_t clearAllPackets();
        uint8_t publishPackets(uint8_t idx);

    public:
        PacketProcessor();
        ~PacketProcessor();
        
        uint8_t initializePacketProcessor(uint8_t newRadarID, InfoConverNode* newPublisher);
        uint8_t processRDIMsg(const ars430_process::RadarPacket::ConstPtr& packet);
        uint8_t processSSPacket(SSPacket_t* packet);
        void SetPublisher(InfoConverNode* newPublisher);
		
        bool loadRDIMessageFromPacket(ars430_process::RadarPacket* newMsg, const ars430_process::RadarPacket::ConstPtr& oldMsg);

        /* Print the index buffer chosen */
        void printPacketGroup(uint8_t idx);
};

#endif /* PROCESS_PACKET_H */

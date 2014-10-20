/*
 * Stopwatch.h
 *
 *  Created on: 29 Sep 2011
 *      Author: thomas
 *
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>

#include <string.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include <unistd.h>
#include <iostream>
#include <map>

#define SEND_INTERVAL_MS 10000

#ifndef DISABLE_STOPWATCH
#define STOPWATCH(name, expression) \
    do \
    { \
        const unsigned long long int startTime = Stopwatch::getInstance().getCurrentSystemTime(); \
        expression \
        const unsigned long long int endTime = Stopwatch::getInstance().getCurrentSystemTime(); \
        Stopwatch::getInstance().addStopwatchTiming(name, endTime - startTime); \
    } \
    while(false)

#define TICK(name) \
    do \
    { \
        Stopwatch::getInstance().tick(name, Stopwatch::getInstance().getCurrentSystemTime()); \
    } \
    while(false)

#define TOCK(name) \
    do \
    { \
        Stopwatch::getInstance().tock(name, Stopwatch::getInstance().getCurrentSystemTime()); \
    } \
    while(false)
#else
#define STOPWATCH(name, expression) \
    expression

#define TOCK(name) ((void)0)

#define TICK(name) ((void)0)

#endif

class Stopwatch
{
    public:
        static Stopwatch & getInstance()
        {
            static Stopwatch instance;
            return instance;
        }

        void addStopwatchTiming(std::string name, unsigned long long int duration)
        {
            if(duration > 0)
            {
                timings[name] = (float)(duration) / 1000.0f;
            }
        }

        void setCustomSignature(unsigned long long int newSignature)
        {
            signature = newSignature;
        }

        const std::map<std::string, float> & getTimings()
        {
            return timings;
        }

        void printAll()
        {
            for(std::map<std::string, float>::const_iterator it = timings.begin(); it != timings.end(); it++)
            {
                std::cout << it->first << ": " << it->second  << "ms" << std::endl;
            }

            std::cout << std::endl;
        }

        void pulse(std::string name)
        {
            timings[name] = 1;
        }

        void sendAll()
        {
            gettimeofday(&clock, 0);

            if((currentSend = (clock.tv_sec * 1000000 + clock.tv_usec)) - lastSend > SEND_INTERVAL_MS)
            {
                int size = 0;
                unsigned char * data = serialiseTimings(size);

                sendto(sockfd, data, size, 0, (struct sockaddr *) &servaddr, sizeof(servaddr));

                free(data);

                lastSend = currentSend;
            }
        }

        static unsigned long long int getCurrentSystemTime()
        {
            timeval tv;
            gettimeofday(&tv, 0);
            unsigned long long int time = (unsigned long long int)(tv.tv_sec * 1000000 + tv.tv_usec);
            return time;
        }

        void tick(std::string name, unsigned long long int start)
        {
        	tickTimings[name] = start;
        }

        void tock(std::string name, unsigned long long int end)
        {
        	float duration = (float)(end - tickTimings[name]) / 1000.0f;

            if(duration > 0)
            {
                timings[name] = duration;
            }
        }

    private:
        Stopwatch()
        {
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
            servaddr.sin_port = htons(45454);
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);

            gettimeofday(&clock, 0);

            signature = clock.tv_sec * 1000000 + clock.tv_usec;

            currentSend = lastSend = clock.tv_sec * 1000000 + clock.tv_usec;
        }

        virtual ~Stopwatch()
        {
            close(sockfd);
        }

        unsigned char * serialiseTimings(int & packetSize)
        {
            packetSize = sizeof(int) + sizeof(unsigned long long int);

            for(std::map<std::string, float>::const_iterator it = timings.begin(); it != timings.end(); it++)
            {
                packetSize += it->first.length() + 1 + sizeof(float);
            }

            int * dataPacket = (int *)calloc(packetSize, sizeof(unsigned char));

            dataPacket[0] = packetSize * sizeof(unsigned char);

            *((unsigned long long int *)&dataPacket[1]) = signature;

            float * valuePointer = (float *)&((unsigned long long int *)&dataPacket[1])[1];

            for(std::map<std::string, float>::const_iterator it = timings.begin(); it != timings.end(); it++)
            {
                valuePointer = (float *)mempcpy(valuePointer, it->first.c_str(), it->first.length() + 1);
                *valuePointer++ = it->second;
            }

            return (unsigned char *)dataPacket;
        }

        timeval clock;
        long long int currentSend, lastSend;
        unsigned long long int signature;
        int sockfd;
        struct sockaddr_in servaddr;
        std::map<std::string, float> timings;
        std::map<std::string, unsigned long long int> tickTimings;
};

#endif /* STOPWATCH_H_ */

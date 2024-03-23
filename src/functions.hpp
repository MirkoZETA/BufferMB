#include <fstream>
#include "./simulator.hpp"
#include <deque>
#include <bits/stdc++.h>
#include <limits.h>
#include <algorithm>

// ############################## Setup Global Variables #################################
int nodes=14;
std::map<std::pair<int, int>, std::vector<int>> shorterIndex;
double median;

std::string bitrateName;

const int bitrateNumber = 5;

// BBP variables
double bitrateCountTotal[bitrateNumber] = {0.0, 0.0, 0.0, 0.0, 0.0};
double bitrateCountBlocked[bitrateNumber] = {0.0, 0.0, 0.0, 0.0, 0.0};
// Weight C+L+S+E:
double meanWeightBitrate[bitrateNumber] = {1.0, 1.83, 3.5, 13.33, 32.83};
// Bitrate map
std::map<float, int> bitRates_map {{ 10.0 , 0 }, { 40.0 , 1 }, { 100.0 , 2 }, { 400.0 , 3 }, {1000.0, 4}};

// Traffic loads
double  lambdas[11] = {1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000};

// Buffer element class
class buffer_element {

  public:
    friend class Buffer;
    buffer_element(int src, int dst, long long id, BitRate *bitRate, double time_arrival){
      this->src = src;
      this->dst = dst;
      this->bitRate = new BitRate(*bitRate);
      this->id = id;
      this->time_arrival = time_arrival;
      this->current_attempts = 1;
    }

    buffer_element(int src, int dst, long long id, BitRate *bitRate, double time_arrival, int attempts){
      this->src = src;
      this->dst = dst;
      this->bitRate = new BitRate(*bitRate);
      this->id = id;
      this->time_arrival = time_arrival;
      this->current_attempts = attempts;
    }

    ~buffer_element() {};

    int src;
    int dst;
    long long id;
    BitRate *bitRate;
    double time_arrival;
    int current_attempts;

    bool operator>(const buffer_element &e) const
    {
      return bitRate->getBitRate() > e.bitRate->getBitRate();
    }

    bool operator<(const buffer_element &e) const
    {
      return bitRate->getBitRate() < e.bitRate->getBitRate();
    }

};

// Buffer class
class Buffer {
  friend class bufer_element;
  public:

    Buffer(){
      this->elements = std::deque<buffer_element>();
      this->last_time = 0;
      this->poped = 0;
      this->pushed = 0;
      this->mean_size_time = 0;
      this->mean_service_time = 0;
    }

    void addElement(buffer_element new_element){
      this->elements.push_back(new_element);
      this->pushed++;
    }

    void pop_front(){
      this->elements.pop_front();
      this->poped++;
    }

    void removeElementAtIndex(int index) {
        if (index < 0 || index >= this->elements.size()) {
            // Manejo de error: índice fuera de rango
            return;
        }
        this->poped++;
        this->elements.erase(this->elements.begin() + index);
    }

    int size(){
      return this->elements.size();
    }

    void clear(){
      this->elements.clear();
    }

    buffer_element *front(){
      return &(this->elements.front());
    }

    buffer_element *back(){
      return &(this->elements.back());
    }

    buffer_element* getElementAtIndex(int index) {
        if (index < 0 || index >= this->elements.size()) {
            // Manejo de error: índice fuera de rango
            return nullptr;
        }
        return &(this->elements[index]);
    }

    std::deque<buffer_element> elements;
    double last_time;
    double mean_size_time;
    double mean_service_time;
    double mean_attempts;

    // Number of connections popped from buffer (allocated succesfully)
    int poped;
    int pushed;

};

// Calculate BBP n/Buffer
double bandwidthBlockingProbability(double bitrateCountTotal[bitrateNumber], 
                                    double bitrateCountBlocked[bitrateNumber],
                                    double meanWeightBitrate[bitrateNumber])
    {
    double BBP = 0;
    double BP = 0;
    double total_weight = 0;

    for (int b = 0; b < bitrateNumber; b++){
        total_weight += meanWeightBitrate[b];
        if (bitrateCountTotal[b] == 0) continue;
        BP = bitrateCountBlocked[b] / bitrateCountTotal[b];
        BBP += meanWeightBitrate[b] * BP;
    }

    return (BBP/total_weight);
}

// Calculate BBP w/buffer
double bandwidthBlockingProbabilityWBuffer(double bitrateCountTotal[bitrateNumber], 
                                           std::deque<buffer_element> buffer,
                                           double meanWeightBitrate[bitrateNumber])
    {
    double BBP = 0;
    double BP = 0;
    double total_weight = 0;

    double count_blocked[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < buffer.size(); i++){
        count_blocked[bitRates_map[buffer[i].bitRate->getBitRate()]] += 1;
    }

    for (int b = 0; b < bitrateNumber; b++){
        total_weight += meanWeightBitrate[b];
        if (count_blocked[b] == 0) continue;
        BP = count_blocked[b] / bitrateCountTotal[b];
        BBP += meanWeightBitrate[b] * BP;
    }

    return (BBP/total_weight);
}

// Result to TXT
void resultsToFile(bool buffer_state, std::fstream &output, double BBP, double BP, int number_connections,
                   int lambda_index, double erlang, Buffer buffer, double last_time, int searchDepth, int maxBufferSize)
{

    // avgService only popped connections
    double avgService = buffer.mean_service_time/buffer.poped;
    double avgAttempts = buffer.mean_attempts/buffer.poped;
    double avgSize = buffer.mean_size_time/buffer.last_time;

    // avgService all poped and blocked connections
    for (int buffElement = 0; buffElement < buffer.size(); buffElement++){
      buffer.mean_service_time += last_time - buffer.getElementAtIndex(buffElement)->time_arrival;
    }

    double avgServiceAll = buffer.mean_service_time/(buffer.poped + buffer.size());

    if (buffer.poped == 0){
      avgService = 0;
      avgServiceAll = 0;
    }



    switch (buffer_state){
        case false:
            // output info to txt:
            output << "N/Buffer erlang index: " << lambda_index
                    << ", erlang: " << erlang
                    << ", general blocking: " << BP
                    << ", BBP: " << BBP                    
                    << '\n';
            break;
        case true:
            if (buffer.size() == 0) std::cout << "\nNo elements in buffer! :P\n";
            // output info to txt:
            output << "W/Buffer erlang index: " << lambda_index
                    << ", erlang: " << erlang
                    << ", BBP: " << BBP 
                    << ", general blocking: " << (buffer.size()/(double)number_connections) 
                    << ", general blocking (original): " << BP
                    << ", buffer size: " << buffer.size() 
                    << ", reallocated: " << buffer.poped 
                    << ", Average try per allocated element: " << avgAttempts
                    << ", Average service time: " << avgService
                    << ", Average service time (ALL): " << avgServiceAll
                    << ", Average buffer size: " << avgSize
                    << ", searchDepth: " << searchDepth
                    << ", maxBufferSize: " << maxBufferSize
                    << ", pushed: " << buffer.pushed
                    << '\n';
            break;
        }
}

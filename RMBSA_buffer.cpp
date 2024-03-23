#include "./src/functions.hpp"


// ############################## Settings #################################
// Simulation
bool verbose = true;
int numberConnections = 1e6; // (requests)
std::vector<char> bandsOrder = {'C', 'L', 'S', 'E'};
int K = INT_MAX; // Number of routes to use

// Buffer settings:
bool buffer_state = true;
int searchDepthSteps[] = {5, 10, 20, 50};  // Define an array with the required steps.
int maxBufferSize = INT_MAX; // Max Buffer Size

// ########################## Variables #################################
// Buffer variables
Buffer buffer;
int searchDepth; // Current Search depth
bool allocating_from_buffer = false;
// Controller and Simulator for acces from UNALLOC
Controller *buffer_controller;
Simulator sim;
// Save last time in simulation
double last_time = 0;
// Traffic
double mu = 1;

// Definition of allocation function
BEGIN_ALLOC_FUNCTION(MB_Alloc) {

    // Update time
    last_time = con.getTimeConnection();

    // Calculating the avg buffer size
    if (buffer_state)
    {
        buffer.mean_size_time += buffer.size() * (con.getTimeConnection() - buffer.last_time);
        buffer.last_time = con.getTimeConnection();
    }

    // Declare variables
    int currentNumberSlots;
    int currentSlotIndex;
    int numberOfSlots;
    int numberOfBands;
    char band;  // Current band
    std::map<char, std::vector<bool>> totalSlots; // Vector of slots for each band
    int bitrateInt = bitRates_map[REQ_BITRATE];

    if (!allocating_from_buffer) bitrateCountTotal[bitrateInt] += 1;

    int nRoutes = std::min((int)NUMBER_OF_ROUTES, K);
    for (int r = 0; r < nRoutes; r++){
        
        for (int bn = 0; bn < NUMBER_OF_BANDS(r, 0); bn++){
            band = bandsOrder[bn];
            totalSlots[band] = std::vector<bool>(LINK_IN_ROUTE(r, 0)->getSlots(band), false);
        }

        int routeLength = 0;
        for (int l = 0; l < NUMBER_OF_LINKS(r); l++) {
            routeLength += LINK_IN_ROUTE(r, l)->getLength();
            for (int bn = 0; bn < NUMBER_OF_BANDS(r, l); bn++) {
                band = bandsOrder[bn];

                for (int s = 0; s < LINK_IN_ROUTE(r, l)->getSlots(band); s++) {
                    totalSlots[band][s] = totalSlots[band][s] | LINK_IN_ROUTE(r, l)->getSlot(s, band);
                }
            }
        }

        for (int m = 0; m < NUMBER_OF_MODULATIONS; m++) {
            // Generate MAP to get the position of the bands inside the current modulation
            std::map<char, int> bandPos = REQ_POS_BANDS(m);

            for (int bn = 0; bn < NUMBER_OF_BANDS(r, 0); bn++) {
                band = bandsOrder[bn];
                int bandIndex = bandPos[band];
                numberOfSlots = REQ_SLOTS_BDM(m, bandIndex);

                if (routeLength > REQ_REACH_BDM(m, bandIndex))
                    continue;

                currentNumberSlots = 0;
                currentSlotIndex = 0;
                for (int s = 0; s < LINK_IN_ROUTE(r, 0)->getSlots(band); s++) {
                    if (totalSlots[band][s] == false) {
                        currentNumberSlots++;
                    }
                    else {
                        currentNumberSlots = 0;
                        currentSlotIndex = s + 1;
                    }
                    if (currentNumberSlots == numberOfSlots) {
                        for (int l = 0; l < NUMBER_OF_LINKS(r); l++) {
                            ALLOC_SLOTS_BDM(LINK_IN_ROUTE_ID(r, l), band, currentSlotIndex, numberOfSlots)
                        }
                        return ALLOCATED;
                    }
                }
            }
        }
    }

    bitrateCountBlocked[bitrateInt] += 1;

    if (buffer_state && !allocating_from_buffer && buffer.size() < maxBufferSize){
        // If the current connection isn't allocated and isnt comming from buffer
        // we add it to queue
        buffer.addElement(buffer_element(SRC, DST, con.getId(), con.getBitrate(), con.getTimeConnection()));
    }
    return NOT_ALLOCATED;
}
END_ALLOC_FUNCTION

BEGIN_UNALLOC_CALLBACK_FUNCTION{

    last_time = t;

    if (buffer_state)
    {
        // Let the alloc function know we are allocating from buffer
        allocating_from_buffer = true;

        for (int be = 0; be < std::min(buffer.size(), searchDepth); be++){
            
            // TODO
            //std::cout << "Current element: " << be << ", Buffer Size: " << buffer.size() <<". \n";

            // For simplicity
            buffer_element *current_element = buffer.getElementAtIndex(be);

            // try to alloc
            //if ((buffer_controller->*(buffer_controller->assignConnection))(front_queue->src, front_queue->dst, *(front_queue->bitRate), front_queue->id, t) == ALLOCATED)
            if ((buffer_controller->*(buffer_controller->assignConnection))
                (current_element->src, current_element->dst, *(current_element->bitRate), current_element->id, t) == ALLOCATED)
            {
                // Add departure to event routine
                sim.addDepartureEvent(current_element->id);

                // Total time the connection was in queue
                buffer.mean_service_time += last_time - current_element->time_arrival;

                // We keep track of how many times attempted to be allocated from buffer
                buffer.mean_attempts += current_element->current_attempts;
                //buffer.mean_attempts += buffer.front()->current_attempts;

                // Element allocated so we poped it and delete() members
                delete (current_element->bitRate);
                buffer.removeElementAtIndex(be);
                // std::cout << "Allocated!\n";

                //std::cout << "Allocated the element on position: " << be << "\n";
                break;
            }
            else
            {
                // If we couldn't allocate from buffer we increase the current attempts
                current_element->current_attempts++;
                //buffer.front()->current_attempts++;
            }
        }
    // Not allocating from buffer anymore
    allocating_from_buffer = false;
    }
}
END_UNALLOC_CALLBACK_FUNCTION

int main(int argc, char* argv[]){

  // Create a new instance of the simulator
    for (int i = 0; i < sizeof(searchDepthSteps) / sizeof(searchDepthSteps[0]); ++i) {
        searchDepth = searchDepthSteps[i];

        std::fstream latex_output;
        if (buffer_state && verbose){
            latex_output.open("./results/latexWBuf_" + std::string(argv[1]) + ".txt", std::ios::out | std::ios::app);
            latex_output << "Search depth: " << searchDepth << std::endl;
        }
        else if (verbose){
            latex_output.open("./results/latexNBuf_" + std::string(argv[1]) + ".txt", std::ios::out | std::ios::app);
            latex_output << "No buffer"<< std::endl;
        }

        for (int lambda = 0; lambda < sizeof(lambdas)/sizeof(double); lambda++) {
            sim = Simulator(std::string("networks/" + std::string(argv[1]) + ".json"), std::string("networks/routes_" + std::string(argv[1]) + ".json"),
                        std::string("networks/bitrates.json"), BDM);

            // Sim parameters
            USE_ALLOC_FUNCTION(MB_Alloc, sim);
            USE_UNALLOC_FUNCTION_BDM(sim);

            sim.setGoalConnections(numberConnections);
            sim.setLambda(lambdas[lambda]);
            sim.setMu(mu);
            sim.init();
            // Set controller accessible for unalloc function (required for buffer)
            buffer_controller = sim.getController();
            sim.run();

            // Save results
            std::fstream output;

            if (verbose)
            {
                std::cout << "Saving results...\n";
                // BBP calculation and output results
                double BBP_results;

                // different BBP formula depending if buffer is activated
                if (buffer_state){
                BBP_results = bandwidthBlockingProbabilityWBuffer(bitrateCountTotal, buffer.elements, meanWeightBitrate);
                output.open("./results/completeBuffer_" + std::string(argv[1]) + ".txt", std::ios::out | std::ios::app);
                latex_output << lambdas[lambda] << "\t" << (buffer.size()/(double)numberConnections)  << "\t\\\\" << std::endl;
                }

                else{
                BBP_results = bandwidthBlockingProbability(bitrateCountTotal, bitrateCountBlocked, meanWeightBitrate);
                output.open("./results/completeNoBuffer_" + std::string(argv[1]) + ".txt", std::ios::out | std::ios::app);
                latex_output << lambdas[lambda] << "\t" << sim.getBlockingProbability()  << "\t\\\\" << std::endl;

                }
                resultsToFile(buffer_state, output, BBP_results, sim.getBlockingProbability(), numberConnections,
                            lambda, lambdas[lambda], buffer, last_time, searchDepth, maxBufferSize);
            }


            // Reset global variables
            // Clear buffer and related variables
            for (int bufferElement = 0; bufferElement < buffer.size(); bufferElement++)
            delete (buffer.elements[bufferElement].bitRate);

            buffer.clear();
            buffer.poped = 0;
            buffer.last_time = 0;
            buffer.mean_service_time = 0;
            buffer.mean_size_time = 0;
            buffer.mean_attempts = 0;
            last_time = 0;

            // Reset global variables for BBP calculation
            for (int b = 0; b < bitrateNumber; b++)
            {
            bitrateCountTotal[b] = 0.0;
            bitrateCountBlocked[b] = 0.0;
            }
            output.close();
        }
        latex_output.close();

        if (!buffer_state)
            break;
    }

  return 0;
}

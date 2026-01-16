#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H

#include <math.h>

#include "convert_type.h"
#include "data_struct.h"

class DataProcess
{
public:
    DataProcess();
    ~DataProcess();

    bool processObjectListMessage(char *in, RadarObjectList *o_list);
    bool processDetectionListMessage(char *in, RadarDetectionList *d_list);
    bool processRadarStatusMessage(char *in, RadarStatus *radar_status);

private:
    ConvertType cvt;
};

#endif 
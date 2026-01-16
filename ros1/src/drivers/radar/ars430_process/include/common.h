/* File: processPacket.h
 *
 * Groups incoming packets into buffer based on timestamp & type
 * Performs simple filtering & thresholding on simeple packet contents
 */

/* General Use Includes */
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>

//Return Values
#define SUCCESS             0
#define NO_DETECTIONS       1
#define PUBLISH_FAIL        2
#define CLEAR_FAIL          3
#define BAD_EVENT_ID        4
#define NO_PUBLISHER        5
#define INIT_FAIL           6
#define BAD_PORT            7
#define BAD_SERVICE_ID      8
#define NO_PUBLISHER        9
#define NO_PUB_CLR_FAIL     10
#define FALSE_DETECTION_1   11
#define FALSE_DETECTION_2   12
#define FALSE_DETECTION_3   13
#define TOO_MUCH_NOISE      14
#define SS_DEFECTIVE_HW     15
#define SS_BAD_VOLT         16
#define SS_BAD_TEMP         17
#define SS_GM_MISSING       18
#define SS_PWR_REDUCED      19
#define NO_PROCESS          20
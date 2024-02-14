#include <glib.h>
#include <glib/gprintf.h>

#include <syslog.h>

#include <axsdk/axevent.h>

#define LOG(fmt, args...)   { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define ERR(fmt, args...)   { syslog(LOG_ERR, fmt, ## args); printf(fmt, ## args); }

#define TOPIC0_TAG  "CameraApplicationPlatform"
#define TOPIC0_NAME "ACAP"
#define TOPIC1_TAG  "GPSData"
#define TOPIC1_NAME "GPS Data Service"
#define EVENT_TAG   "GPSCoordinates"
#define EVENT_NAME  "GPS Coordinate Event"

static AXEventHandler *event_handler = 0;
static guint event_id = 0;

void metadata_stream_init()
{
    AXEventKeyValueSet *dataSet = NULL;

    event_handler = ax_event_handler_new();
    dataSet = ax_event_key_value_set_new();

    //Note that the namespace is "tnsaxis:".  It is not recommended to create own name spaces or use the
    //the ONVIF namespace "tns1:"

    //TOPIC LEVEL 0 
    ax_event_key_value_set_add_key_value( dataSet,"topic0", "tnsaxis", TOPIC0_TAG, AX_VALUE_TYPE_STRING,NULL);
    //ax_event_key_value_set_add_nice_names( dataSet,"topic0", "tnsaxis", TOPIC0_NAME, TOPIC0_NAME ,NULL);
    //As we are using the standard CameraApplicationPlatform there is no need to set nice name  

    //TOPIC LEVEL 1
    ax_event_key_value_set_add_key_value( dataSet,"topic1", "tnsaxis", TOPIC1_TAG, AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_add_nice_names( dataSet,"topic1", "tnsaxis", TOPIC1_TAG, TOPIC1_NAME, NULL);

    //TOPIC LEVEL 2
    ax_event_key_value_set_add_key_value(  dataSet, "topic2", "tnsaxis", EVENT_TAG , AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_add_nice_names( dataSet, "topic2", "tnsaxis", EVENT_TAG, EVENT_NAME, NULL);
    // A data event is typically used for a specific client/application that knows how to process the data.
    // If the event is not intended to trigger general actions it is a good idea to the tag the event
    // with "isApplicationData" to requests clients/actionrules not to display the event.
    ax_event_key_value_set_mark_as_user_defined( dataSet, "topic2", "tnsaxis", "isApplicationData", NULL);

    //EVENT DATA INSTANCE
    // A tag id that holds the user data.  It is recommended to mark event user data with "isApplicationData"
    ax_event_key_value_set_add_key_value( dataSet, "Latitude", NULL, "" , AX_VALUE_TYPE_DOUBLE,NULL);
    ax_event_key_value_set_mark_as_data(  dataSet, "Latitude", NULL, NULL);
    ax_event_key_value_set_mark_as_user_defined( dataSet, "Latitude", NULL, "isApplicationData", NULL);

    ax_event_key_value_set_add_key_value( dataSet, "Latitude Reference", NULL, "" , AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_mark_as_data(  dataSet, "Latitude Reference", NULL, NULL);
    ax_event_key_value_set_mark_as_user_defined( dataSet, "Latitude Reference", NULL, "isApplicationData", NULL);

    ax_event_key_value_set_add_key_value( dataSet, "Longitude", NULL, "" , AX_VALUE_TYPE_DOUBLE,NULL);
    ax_event_key_value_set_mark_as_data(  dataSet, "Longitude", NULL, NULL);
    ax_event_key_value_set_mark_as_user_defined( dataSet, "Longitude", NULL, "isApplicationData", NULL);

    ax_event_key_value_set_add_key_value( dataSet, "Longitude Reference", NULL, "" , AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_mark_as_data(  dataSet, "Longitude Reference", NULL, NULL);
    ax_event_key_value_set_mark_as_user_defined( dataSet, "Longitude Reference", NULL, "isApplicationData", NULL);

    ax_event_key_value_set_add_key_value( dataSet, "Speed", NULL, "" , AX_VALUE_TYPE_DOUBLE,NULL);
    ax_event_key_value_set_mark_as_data(  dataSet, "Speed", NULL, NULL);
    ax_event_key_value_set_mark_as_user_defined( dataSet, "Speed", NULL, "isApplicationData", NULL);

    ax_event_key_value_set_add_key_value( dataSet, "Speed Unit", NULL, "" , AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_mark_as_data(  dataSet, "Speed Unit", NULL, NULL);
    ax_event_key_value_set_mark_as_user_defined( dataSet, "Speed Unit", NULL, "isApplicationData", NULL);

    ax_event_key_value_set_add_key_value( dataSet, "Altitude", NULL, "" , AX_VALUE_TYPE_DOUBLE,NULL);
    ax_event_key_value_set_mark_as_data(  dataSet, "Altitude", NULL, NULL);
    ax_event_key_value_set_mark_as_user_defined( dataSet, "Altitude", NULL, "isApplicationData", NULL);

    ax_event_key_value_set_add_key_value( dataSet, "Altitude Reference", NULL, "" , AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_mark_as_data(  dataSet, "Altitude Reference", NULL, NULL);
    ax_event_key_value_set_mark_as_user_defined( dataSet, "Altitude Reference", NULL, "isApplicationData", NULL);

    //Note that the 3:rd parameter defines if the event is stateful or stateless.  1 = stateless, 0 = stateful
    if( !ax_event_handler_declare( event_handler, dataSet, 1, &event_id, NULL, NULL, NULL) ) {
        ERR("Could not declare event\n");
    }

    ax_event_key_value_set_free( dataSet );

}

void metadata_send(double lat, const char lat_ref, 
           double lon, const char lon_ref,
           double speed, const char *speed_unit,
           double alt, int alt_ref)
{ 
    AXEventKeyValueSet *dataSet = NULL;
    AXEvent            *event = NULL;
    GTimeVal           time_stamp;

    char *latrefs = g_strdup_printf("%c", lat_ref);
    char *lonrefs = g_strdup_printf("%c", lon_ref);
    char *altrefs;

    if (alt_ref == 1) {
        altrefs = g_strdup_printf("Below Sea Level");
    } else {
        altrefs = "Above Sea Level";
    }

    dataSet = ax_event_key_value_set_new();
    ax_event_key_value_set_add_key_value( dataSet,"Latitude",NULL, &lat, AX_VALUE_TYPE_DOUBLE,NULL);
    ax_event_key_value_set_add_key_value( dataSet,"Latitude Reference",NULL, latrefs, AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_add_key_value( dataSet,"Longitude",NULL, &lon, AX_VALUE_TYPE_DOUBLE,NULL);
    ax_event_key_value_set_add_key_value( dataSet,"Longitude Reference",NULL, lonrefs, AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_add_key_value( dataSet,"Speed",NULL, &speed, AX_VALUE_TYPE_DOUBLE,NULL);
    ax_event_key_value_set_add_key_value( dataSet,"Speed Unit",NULL, speed_unit, AX_VALUE_TYPE_STRING,NULL);
    ax_event_key_value_set_add_key_value( dataSet,"Altitude",NULL, &alt, AX_VALUE_TYPE_DOUBLE,NULL);
    ax_event_key_value_set_add_key_value( dataSet,"Altitude Reference",NULL, altrefs, AX_VALUE_TYPE_STRING,NULL);

    g_get_current_time(&time_stamp);
    event = ax_event_new( dataSet, &time_stamp);
    ax_event_key_value_set_free( dataSet );

    if( !ax_event_handler_send_event( event_handler, event_id, event, NULL) ) {
        ERR("Could not fire event\n");
    }

    g_free(latrefs);
    g_free(lonrefs);
    ax_event_free(event);
}

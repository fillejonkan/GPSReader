#ifndef INCLUSION_GUARD_METADATA_STREAM_H
#define INCLUSION_GUARD_METADATA_STREAM_H

/* Initialize metadata streaming functionality */
void metadata_stream_init();

/* Send GPS metadata to Event Stream */
void metadata_send(double lat, const char lat_ref, 
                   double lon, const char lon_ref,
                   double speed, const char *speed_unit,
                   double alt, int alt_ref);


#endif // INCLUSION_METADATA_STREAM_H
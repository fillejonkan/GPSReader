#ifndef INCLUSION_GUARD_OVERLAY_H
#define INCLUSION_GUARD_OVERLAY_H

/** @file overlay.h
 * @Brief Responsible for updating overlays with metadata information
 *
 */

/**
 * Forward-declared handle for MDP object.
 */
typedef struct overlay* overlay_handle;

/**
 */
overlay_handle overlay_init();

/**
 * Cleanup Metadata Push framework and deallocate resources.
 *
 * @return No return value.
 */
void overlay_cleanup(overlay_handle *handle_p);

/**
 * Send Metadata to ACS.
 *
 * @param data the set of strings to update the overlay with
 *
 * @return TRUE on success, FALSE on any kind of error.
 */
gboolean overlay_set_data(const overlay_handle handle,
						  GList *metadata_items, 
						  const char *utc_time,
						  const char *analytic);

#endif // INCLUSION_GUARD_OVERLAY_H
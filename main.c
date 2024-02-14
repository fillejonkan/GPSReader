#include <stdlib.h>
#include <glib.h>
#include <glib/gprintf.h>

#include <stdio.h>
#include <syslog.h>
#include <signal.h>
#include <string.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <math.h>
#include <time.h>

//#include <statuscache.h>

#include "vdo-error.h"
#include "vdo-map.h"
#include "vdo-stream.h"
#include "vdo-types.h"
#include <axsdk/axevent.h>
#include <exif-data.h>

#include <nmea/nmea.h>
#include "camera/camera.h"
#include "sqlite3/sqlite3.h"

#include "ftplib.h"
#include "metadata_stream.h"

#define LOG(fmt, args...)   { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define ERR(fmt, args...)   { syslog(LOG_ERR, fmt, ## args); printf(fmt, ## args); }
#define APP_ID              "gpsrecv"
#define APP_NICE_NAME       "AXIS GPS Receiver"

#define UDP_SERVER_PORT     (8300)
#define UDP_BUFF_SIZE       (4096)

#define EXIF_INTERVAL       (5)
#define GPS_PRECISION       (1000000)
#define OVERLAY_BUF_SIZE    (64)
#define OVERLAY_STR_SIZE    (OVERLAY_BUF_SIZE -1)

/* byte order to use in the EXIF block (use same as IPhone) */
#define FILE_BYTE_ORDER EXIF_BYTE_ORDER_MOTOROLA

/* comment to write into the EXIF block */
#define FILE_COMMENT "AXIS Network Camera Snapshot"

/* special header required for EXIF_TAG_USER_COMMENT */
#define ASCII_COMMENT "ASCII\0\0\0"

#define FILE_NAME "/tmp/exif.jpg"

#define DEFAULT_FTP_USER "root"
#define DEFAULT_FTP_PASS "pass"
#define DEFAULT_FTP_SERVER "192.168.1.20"
#define DEFAULT_FTP_FOLDER "axis-folder"
#define DEFAULT_FTP_BASENAME "axis-gps-image"

static unsigned char udp_rcv[UDP_BUFF_SIZE] = {0,};

static int event_source_exif = 0;

static char* HEADINGS[16] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                                "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
                             };
static double HEADING_LIMITS[16] = { 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180,
                                     202.5, 225, 247.5, 270, 292.5, 315, 337.5,
                                     360
                                   };

static int udp_port = UDP_SERVER_PORT;
static int exif_interval = EXIF_INTERVAL;
static char *ftp_user = NULL;
static char *ftp_pass = NULL;
static char *ftp_server = NULL;
static char *ftp_folder = NULL;
static char *ftp_basename = NULL;

/* raw EXIF header data */
static const unsigned char exif_header[] = {
  0xff, 0xd8, 0xff, 0xe1
};
/* length of data in exif_header */
static const unsigned int exif_header_len = sizeof(exif_header);

static GMainLoop *loop;
static gboolean exit_thread;
static GCond new_interval_c;
static GMutex new_interval_m;
static gchar *googleMapsAPIKey = NULL;
CAMERA_HTTP_Reply http_g;

typedef struct {
  GMutex m;
  int interval;
  int max_lines;
  int lines;
} GPSLog;

static GPSLog *gps_log;

typedef struct {
  GMutex m;
  double lat;
  double lon;
  double speed;
  double elv;
  double direction;
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
  int fix;
  gboolean inited;
} GPSPosition;

static GPSPosition *curr_position;

typedef struct {
  GMutex m;
  sqlite3* db;
} SQLDB;

struct nmea_udp_info {
    int udp_socket;
    nmeaPARSER parser;
    GIOChannel *channel;
    guint event_source;
};

struct nmea_udp_info nmea_udp = {.udp_socket = -1,
                                 .channel = NULL,
                                 .event_source = 0};

static VdoStream* stream;

static SQLDB *sql;

static VdoStream* open_stream();
static void update_dynamic_overlay(char *s);
static void update_overlay(double d_lat, double d_lon, double direction, 
                           double speed);

static void safe_unlock_mutex(GMutex *m, char *caller);

static void handle_sigterm(int signo);
static void init_signals();

static gchar *new_string(gchar *str, ...);
static void free_string(gchar* str);

static int get_line_count(void *p_data, int num_fields, char **p_fields, 
                          char **p_col_names);
static void sql_stmt(const char* stmt);
static int sql_log(void *p_data, int num_fields, char **p_fields, 
                   char **p_col_names);

static char *get_heading(double deg);
static double DMs_to_d(double DMs);

static void set_api_key(const char *value);
static void set_udp_port(const char *value);
static void set_exif_int(const char *value);
static void set_ftp_user(const char *value);
static void set_ftp_pass(const char *value);
static void set_ftp_server(const char *value);
static void set_ftp_folder(const char *value);
static void set_ftp_basename(const char *value);

static void set_log_interval(const char *value);
static void set_log_count(const char *value);
static void api_settings_get(CAMERA_HTTP_Reply http, 
                             CAMERA_HTTP_Options options);
static void api_settings_set(CAMERA_HTTP_Reply http,
                             CAMERA_HTTP_Options options);
static void api_position_now(CAMERA_HTTP_Reply http,
                             CAMERA_HTTP_Options options);
static void api_position_history(CAMERA_HTTP_Reply http,
                                 CAMERA_HTTP_Options options);
static void *log_position(void *data);

static ExifEntry *init_tag(ExifData *exif, ExifIfd ifd, ExifTag tag);
static ExifEntry *create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag,
                             size_t len);
static void upload_ftp_image(const char *file_name);
static gboolean exif_timeout(gpointer data);

static gboolean udp_callback(GIOChannel *source,
                             GIOCondition cond,
                             gpointer data);
static int udp_init();
void udp_destroy();

int udp_init()
{
  struct sockaddr_in si_me;
  int udp_socket;

  if ((udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
    LOG("Could not create socket!\n");
    return -1;
  }

  memset((char *) &si_me, 0, sizeof(si_me));

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(udp_port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(udp_socket, (const struct sockaddr *) &si_me, sizeof(si_me))==-1) {
    LOG("Failed to bind socket!\n");
    return -1;
  }

  int reuse = 1;
  setsockopt(udp_socket,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(int));

  LOG("Listening on socket %d, UDP port %d for incoming connections...\n",
    udp_socket, udp_port);

  GIOChannel *udp_channel = g_io_channel_unix_new(udp_socket);

  nmea_udp.udp_socket = udp_socket;
  nmea_udp.channel = udp_channel;
  nmea_parser_init(&nmea_udp.parser);

  nmea_udp.event_source = 
    g_io_add_watch(udp_channel, G_IO_IN, (GIOFunc) udp_callback, NULL);

  return 0;
}

void udp_destroy()
{
    /* TODO: Despite all this, initial UDP server seems to stay alive. */
    LOG("Destroying UDP server\n");

    if (nmea_udp.channel) {
        LOG("Shutting down IO channel\n");
        g_source_remove(nmea_udp.event_source);
        g_io_channel_shutdown(nmea_udp.channel, TRUE, NULL);
        g_io_channel_unref(nmea_udp.channel);
    }

    if (nmea_udp.udp_socket != -1) {
        LOG("Closing socket %d\n", nmea_udp.udp_socket);
        shutdown(nmea_udp.udp_socket, 2);
        close(nmea_udp.udp_socket);
        nmea_udp.udp_socket = -1;
    }

    nmea_parser_destroy(&nmea_udp.parser);
}

gboolean udp_callback(GIOChannel *source,
                      GIOCondition cond,
                      gpointer data)
{
  gsize bytes_read;
  
  int s = nmea_udp.udp_socket;

  // TODO: Can remove?
  memset(udp_rcv, 0, sizeof(udp_rcv));
  
  bytes_read = recvfrom(s, 
                        udp_rcv, 
                        sizeof(udp_rcv), 
                        0, 
                        NULL, 
                        0);

  if (bytes_read == -1) {
    LOG("Failed to receive data!\n");
    return TRUE;
  }

  nmeaINFO info;

  nmea_zero_INFO(&info);
  
  double d_lat, d_lon;
  
  nmea_parse(&nmea_udp.parser, (char *)udp_rcv, sizeof(udp_rcv), &info);
    
  d_lat = DMs_to_d(info.lat);
  d_lon = DMs_to_d(info.lon);
    
  if (info.fix != 1) {
    g_mutex_lock(&curr_position->m);
    curr_position->lat = d_lat;
    curr_position->lon = d_lon;
    curr_position->speed = info.speed;
    curr_position->direction = info.direction;
    curr_position->elv = info.elv;

    curr_position->year = info.utc.year + 1900;
    curr_position->month = info.utc.mon + 1;
    curr_position->day = info.utc.day;
    curr_position->hour = info.utc.hour;
    curr_position->minute = info.utc.min;
    curr_position->second = info.utc.sec;

    curr_position->fix = info.fix;
    curr_position->inited = TRUE;

    safe_unlock_mutex(&curr_position->m, "udp_callback");
    update_overlay(d_lat, d_lon, info.direction, info.speed);
  } else {
    ERR("Invalid GPS fix, don't update value\n");
  }

  return TRUE;
}

static void update_dynamic_overlay(char *s)
{
    if (strlen(s) > OVERLAY_STR_SIZE) {
        g_message("Overlay string to large %d > %d", strlen(s),
            OVERLAY_STR_SIZE);
        return;
    }

#if 0
    /* Ignore return value if group is already created */
    sc_create_group("DYNAMIC_TEXT_IS1", 512, 0);

    struct sc_param sc_par = { .name="DYNAMIC_TEXT",
                               .size=OVERLAY_BUF_SIZE,
                               .data=s,
                               .type=SC_STRING};

    /* NULL-terminated list of pointers to param structs. we need just one */
    struct sc_param *arr[2] = {&sc_par, 0};

    sc_set_group("DYNAMIC_TEXT_IS1", arr, SC_CREATE);
#endif
}


/* To avoid attempt to unlock already unlocked mutexes */
static void safe_unlock_mutex(GMutex *m, char *caller)
{
  //LOG("Result trylock: %d caller: %s\n", g_mutex_trylock(m), caller);
  g_mutex_unlock(m);  
}

static void handle_sigterm(int signo)
{
  g_main_loop_quit(loop);
}

static void init_signals()
{
  struct sigaction sa;
  sa.sa_flags = 0;

  sigemptyset(&sa.sa_mask);
  sa.sa_handler = handle_sigterm;
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGINT, &sa, NULL);
}

static gchar *new_string(gchar *str, ...)
{
  va_list ap;
  gchar *tmp_str;
 
  va_start(ap, str);
  tmp_str = g_strdup_vprintf(str, ap);
  
  va_end(ap);
  return tmp_str;
}

static void free_string(gchar* str) 
{
  if(str != NULL)
    g_free(str);
}

static char *get_heading(double deg)
{
  int i = 0;
  deg = ((int)(deg + 11.25)) % 360;
  while(deg > HEADING_LIMITS[i]) {
    i = i + 1;
  }
  return HEADINGS[i];
}

static int get_line_count(void *p_data, int num_fields, char **p_fields, 
                          char **p_col_names) {
  g_mutex_lock(&gps_log->m);
  gps_log->lines = atoi(p_fields[0]);
  //g_mutex_unlock(&gps_log->m);
  safe_unlock_mutex(&gps_log->m, "get_line_count");
  return 0;
}

static void sql_stmt(const char* stmt) {
  char *errmsg;
  int   ret;

  g_mutex_lock(&sql->m);
  ret = sqlite3_exec(sql->db, stmt, 0, 0, &errmsg);
  //g_mutex_unlock(&sql->m);
  safe_unlock_mutex(&sql->m, "sql_stmt");
  
  if(ret != SQLITE_OK) {
    printf("Error in statement: %s [%s].\n", stmt, errmsg);
  }
}

/*
 * Input +/-[degree][min].[sec/60]
 */
static double DMs_to_d(double DMs)
{
  gboolean is_neg = DMs < 0;
  if(is_neg) {
    DMs = -1 * DMs;
  }
  double D = trunc(DMs / 100);
  double M = trunc(DMs - 100*D);
  double s = (DMs - 100*D - M) * 60;
  double d = D + M/60 + s/3600;
  
  return is_neg ? -1*d : d;
}

static void update_overlay(double d_lat, double d_lon, double direction, 
                           double speed)
{
  char H_lat = 'N', H_lon = 'E';
  if(d_lat < 0) {
    d_lat = -1*d_lat;
    H_lat = 'S';
  }
  if(d_lon < 0) {
    d_lon = -1*d_lon;
    H_lon = 'W';
  }
  
  char *ovl_text = new_string("%f%c %f%c, Heading %s at %.1f mph", 
                              d_lat, H_lat, d_lon, H_lon,
                              get_heading(direction), speed * 0.621371);

  update_dynamic_overlay(ovl_text);
  free_string(ovl_text);
}

static void set_api_key(const char *value)
{
    if(googleMapsAPIKey != NULL) 
    {
        g_free(googleMapsAPIKey);
    }
    
    googleMapsAPIKey = strdup(value);
}

static void set_udp_port(const char *value)
{
    int port;

    if (sscanf(value, "%d", &port) != 1) {
        LOG("Invalid UDP port number, use default\n");
    }

    LOG("Got port number %d\n", port);

    port = CLAMP(port, 1, 65535);

    if (port != udp_port) {
        LOG("UDP Port number changed (%d -> %d)\n", udp_port, port);

        udp_port = port;

        /* restart UDP server if it is up and running */
        if (nmea_udp.udp_socket != -1) {
            udp_destroy();
            udp_init();
        }
    }
}

static void set_exif_int(const char *value)
{
    int exif;

    if (sscanf(value, "%d", &exif) != 1) {
        LOG("Invalid EXIF interval\n");
    }

    LOG("Got EXIF interval %d\n", exif);

    exif = CLAMP(exif, 1, 3600);

    if (exif_interval != exif && event_source_exif != 0) {
        LOG("Changing timer for EXIF upload (%d -> %d)\n", exif_interval,
            exif);

        g_source_remove(event_source_exif);

        event_source_exif = g_timeout_add_seconds(exif,
            exif_timeout, NULL);
    }

    exif_interval = exif;
}

static void set_ftp_user(const char *value)
{
    g_free(ftp_user);
    ftp_user = g_strdup(value);

    LOG("Got new FTP user name %s\n", ftp_user);
}

static void set_ftp_pass(const char *value)
{
    g_free(ftp_pass);
    ftp_pass = g_strdup(value);

    LOG("Got new FTP password %s\n", ftp_pass);
}

static void set_ftp_server(const char *value)
{
    g_free(ftp_server);
    ftp_server = g_strdup(value);

    LOG("Got new FTP server %s\n", ftp_server);
}

static void set_ftp_folder(const char *value)
{
    g_free(ftp_folder);
    ftp_folder = g_strdup(value);

    LOG("Got new FTP folder %s\n", ftp_folder);
}

static void set_ftp_basename(const char *value)
{
    g_free(ftp_basename);
    ftp_basename = g_strdup(value);

    LOG("Got new FTP base name %s\n", ftp_basename);
}

static void set_log_interval(const char *value)
{
  LOG("Updating log interval\n");  
  g_mutex_lock(&gps_log->m);
  gps_log->interval = atoi(value);
  //g_mutex_unlock(&gps_log->m);
  safe_unlock_mutex(&gps_log->m, "set_log_interval");
  g_cond_signal(&new_interval_c);
}

static void set_log_count(const char *value)
{
  LOG("Updating log count\n");
  g_mutex_lock(&gps_log->m);
  gps_log->max_lines = atoi(value);
  
  if(gps_log->max_lines < gps_log->lines) {
    printf("Too many lines, removing the %d oldest\n", gps_log->lines - gps_log->max_lines);
    char *query = new_string("DELETE FROM gps_history WHERE time NOT IN (SELECT time FROM gps_history ORDER BY time DESC LIMIT %d)", gps_log->max_lines);
    sql_stmt(query);
    free_string(query);
  }
  //g_mutex_unlock(&gps_log->m);
  safe_unlock_mutex(&gps_log->m, "set_log_count");
}

static void api_settings_get(CAMERA_HTTP_Reply http, 
                             CAMERA_HTTP_Options options)
{
  camera_http_sendXMLheader(http);
  camera_http_output(http, "<settings>");
  g_mutex_lock(&gps_log->m);
  camera_http_output(http, "<param name='LogInterval' value='%d'/>", gps_log->interval);
  camera_http_output(http, "<param name='LogCount' value='%d'/>", gps_log->max_lines);
  camera_http_output(http, "<param name='ApiKey' value='%s'/>", googleMapsAPIKey);
  camera_http_output(http, "<param name='UDPPort' value='%d'/>", udp_port);
  camera_http_output(http, "<param name='EXIFInt' value='%d'/>", exif_interval);
  camera_http_output(http, "<param name='FTPUser' value='%s'/>", ftp_user);
  camera_http_output(http, "<param name='FTPPass' value='%s'/>", ftp_pass);
  camera_http_output(http, "<param name='FTPServer' value='%s'/>", ftp_server);
  camera_http_output(http, "<param name='FTPFolder' value='%s'/>", ftp_folder);
  camera_http_output(http, "<param name='FTPBaseName' value='%s'/>", ftp_basename);
  safe_unlock_mutex(&gps_log->m, "api_settings_get");
  camera_http_output(http, "</settings>");
}

static void api_settings_set(CAMERA_HTTP_Reply http,
                             CAMERA_HTTP_Options options)
{
  const char *value;
  const char *param;

  camera_http_sendXMLheader(http);

  param = camera_http_getOptionByName(options, "param");
  value = camera_http_getOptionByName(options, "value");

  if(!(param && value)) {
    camera_http_output(http, "<error description='Syntax: param or value missing'/>");
    ERR("api_settings_set: param or value is missing\n");
    return;
  }

  if(!camera_param_set(param, value)) {
    camera_http_output(http, "<error description='Could not set %s to %s'/>",param, value);
    ERR("api_settings_set: Could not set %s to %s\n", param, value);
    return;
  }
  camera_http_output(http, "<success/>");
}

static void api_position_now(CAMERA_HTTP_Reply http,
                             CAMERA_HTTP_Options options)
{
  printf("Asking for position\n");
  camera_http_output(http, "Content-Type: application/vnd.google-earth.kml+xml; Cache-Control: no-cache\r\n\r\n");
  camera_http_output(http, "<?xml version=\"1.0\"?>\r\n");
  camera_http_output(http, "<kml xmlns=\"http://earth.google.com/kml/2.2\">\r\n");
  camera_http_output(http, "<Placemark>\r\n");
  camera_http_output(http, "<name>Current Location</name>\r\n");
  camera_http_output(http, "<Point>\r\n");
  g_mutex_lock(&curr_position->m);
  camera_http_output(http, "<coordinates>%f,%f</coordinates>\r\n", curr_position->lon, curr_position->lat);
  //g_mutex_unlock(&curr_position->m);
  safe_unlock_mutex(&curr_position->m, "api_position_now");
  camera_http_output(http, "</Point>\r\n");
  camera_http_output(http, "</Placemark>\r\n");
  camera_http_output(http, "</kml>\r\n");
}

static int sql_log(void *p_data, int num_fields, char **p_fields, 
                   char **p_col_names) {

  camera_http_output(http_g, "%s,%s\r\n", p_fields[2], p_fields[1]);
  return 0;
}

static void api_position_history(CAMERA_HTTP_Reply http,
                                 CAMERA_HTTP_Options options)
{
  camera_http_output(http, "Content-Type: application/vnd.google-earth.kml+xml; Cache-Control: no-cache\r\n\r\n");
  camera_http_output(http, "<?xml version=\"1.0\"?>\r\n");
  camera_http_output(http, "<kml xmlns=\"http://earth.google.com/kml/2.2\">\r\n");
  camera_http_output(http, "<Document>\r\n");
  camera_http_output(http, "<name>gps.log</name>\r\n");
  camera_http_output(http, "<Style id=\"style_line\">\r\n");
  camera_http_output(http, "<LineStyle>\r\n");
  camera_http_output(http, "<color>73FF0000</color><width>5</width>\r\n");
  camera_http_output(http, "</LineStyle>\r\n");
  camera_http_output(http, "</Style>\r\n");
  camera_http_output(http, "<Placemark>\r\n");
  camera_http_output(http, "<name>Path</name>\r\n");
  camera_http_output(http, "<styleUrl>#style_line</styleUrl>\r\n");
  camera_http_output(http, "<LineString>\r\n");
  camera_http_output(http, "<tessellate>1</tessellate>\r\n");
  camera_http_output(http, "<coordinates>\r\n");
  
  http_g = http;
  g_mutex_lock(&sql->m);
  sqlite3_exec(sql->db, "select * from gps_history order by time asc", sql_log, NULL, NULL);
  //g_mutex_unlock(&sql->m);
  safe_unlock_mutex(&sql->m, "api_position_history 1");

  g_mutex_lock(&curr_position->m);
  camera_http_output(http, "%f,%f\r\n", curr_position->lon, curr_position->lat);
  camera_http_output(http, "</coordinates>\r\n");
  camera_http_output(http, "</LineString>\r\n");
  camera_http_output(http, "</Placemark>\r\n");  
  camera_http_output(http, "<Placemark>\r\n");
  camera_http_output(http, "<name>Current Location</name>\r\n");
  camera_http_output(http, "<Point>\r\n");
  camera_http_output(http, "<coordinates>%f,%f</coordinates>\r\n", curr_position->lon, curr_position->lat);
  //g_mutex_unlock(&curr_position->m);
  safe_unlock_mutex(&curr_position->m, "api_position_history 2");
  camera_http_output(http, "</Point>\r\n");
  camera_http_output(http, "</Placemark>\r\n");
  camera_http_output(http, "</Document>\r\n");
  camera_http_output(http, "</kml>\r\n");
}

/* Get an existing tag, or create one if it doesn't exist */
static ExifEntry *init_tag(ExifData *exif, ExifIfd ifd, ExifTag tag)
{
  ExifEntry *entry;
  /* Return an existing tag if one exists */
  if (!((entry = exif_content_get_entry (exif->ifd[ifd], tag)))) {
      /* Allocate a new entry */
      entry = exif_entry_new ();
      assert(entry != NULL); /* catch an out of memory condition */
      entry->tag = tag; /* tag must be set before calling
         exif_content_add_entry */

      /* Attach the ExifEntry to an IFD */
      exif_content_add_entry (exif->ifd[ifd], entry);

      /* Allocate memory for the entry and fill with default data */
      exif_entry_initialize (entry, tag);

      /* Ownership of the ExifEntry has now been passed to the IFD.
       * One must be very careful in accessing a structure after
       * unref'ing it; in this case, we know "entry" won't be freed
       * because the reference count was bumped when it was added to
       * the IFD.
       */
      exif_entry_unref(entry);
  }
  return entry;
}

/* Create a brand-new tag with a data field of the given length, in the
 * given IFD. This is needed when exif_entry_initialize() isn't able to create
 * this type of tag itself, or the default data length it creates isn't the
 * correct length.
 */
static ExifEntry *create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag,
                             size_t len)
{
  void *buf;
  ExifEntry *entry;
  
  /* Create a memory allocator to manage this ExifEntry */
  ExifMem *mem = exif_mem_new_default();
  assert(mem != NULL); /* catch an out of memory condition */

  /* Create a new ExifEntry using our allocator */
  entry = exif_entry_new_mem (mem);
  assert(entry != NULL);

  /* Allocate memory to use for holding the tag data */
  buf = exif_mem_alloc(mem, len);
  assert(buf != NULL);

  /* Fill in the entry */
  entry->data = buf;
  entry->size = len;
  entry->tag = tag;
  entry->components = len;
  entry->format = EXIF_FORMAT_UNDEFINED;

  /* Attach the ExifEntry to an IFD */
  exif_content_add_entry (exif->ifd[ifd], entry);

  /* The ExifMem and ExifEntry are now owned elsewhere */
  exif_mem_unref(mem);
  exif_entry_unref(entry);

  return entry;
}

static void upload_ftp_image(const char *file_name)
{
  netbuf *nControl = NULL;

  int result = FtpConnect(ftp_server, &nControl);

  if (!result) {
    ERR("Failed to connect to %s\n", ftp_server);
    return;
  }

  LOG("Connected to host %s, netbuf %x\n", ftp_server, (unsigned)nControl);

  result = FtpLogin(ftp_user, ftp_pass, nControl);

   if (!result) {
    ERR("failed to login %s\n", ftp_server);
    return;
  }

  char time_buf[80];
  time_t rawtime;
  struct tm *timeinfo;

  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(time_buf, 80, "%Y-%m-%d_%H_%M_%S", timeinfo);
  
  char *filename_upload = g_strdup_printf("%s/%s-%s.jpg", 
    ftp_folder, ftp_basename, time_buf);

  LOG("Uploading file %s\n", filename_upload);

  result = FtpPut(file_name, filename_upload, FTPLIB_IMAGE, nControl);
  g_free(filename_upload);

  if (!result) {
    ERR("Failed to upload image %s\n", file_name);
    return;
  }

  LOG("Closing connection...\n");

  FtpClose(nControl);
}

static int width, height;

/*
 * Handle timeout for EXIF upload
 */
static gboolean exif_timeout(gpointer data)
{
    double lat;
    double lon;
    double speed;
    double elv;
    double direction;
    int init;

    int utc_year;
    int utc_month;
    int utc_day;
    int utc_hour;
    int utc_minute;
    int utc_second;
    GError *error = NULL;

    void *image_data;

    ExifEntry *entry;
    ExifData *exif = exif_data_new();
    unsigned char *exif_data;
    unsigned int exif_data_len;

    g_mutex_lock(&curr_position->m);
        lat = curr_position->lat;
        lon = curr_position->lon;
        speed = curr_position->speed;
        direction = curr_position->direction;
        elv = curr_position->elv;
        init = curr_position->inited;

        utc_year = curr_position->year;
        utc_month = curr_position->month;
        utc_day = curr_position->day;
        utc_hour = curr_position->hour;
        utc_minute = curr_position->minute;
        utc_second = curr_position->second;

    safe_unlock_mutex(&curr_position->m, "exif_timeout");

    /* Update overlay to create better sync between snapshot and metadata */
    update_overlay(lat, lon, direction, speed);

    if (!init) {
        LOG("GPS value not initialized, do nothing!\n");
        return TRUE;
    }

    if (!stream) {
      stream = open_stream();

      if (!stream) {
        ERR("Failed to allocate stream!\n");
        return TRUE;
      }
    }

    LOG("Getting frame %x\n", (int) stream);

    VdoBuffer* buffer = vdo_stream_get_buffer(stream, &error);
    VdoFrame* frame   = vdo_buffer_get_frame(buffer);

    if (!frame) {
      return TRUE;
    }

    image_data = vdo_buffer_get_data(buffer);
    size_t data_size = vdo_frame_get_size(frame);

    LOG("Got frame\n");

    /* Set the image options */
    exif_data_set_option(exif, EXIF_DATA_OPTION_FOLLOW_SPECIFICATION);
    exif_data_set_data_type(exif, EXIF_DATA_TYPE_COMPRESSED);
    exif_data_set_byte_order(exif, FILE_BYTE_ORDER);

    /* Create the mandatory EXIF fields with default data */
    exif_data_fix(exif);

    double abs_lat = lat;
    double abs_lon = lon;

    unsigned char lat_ref = 'N';
    if (lat < 0) {
      abs_lat = -lat;
      lat_ref = 'S';
    }

    unsigned char lon_ref = 'E';
    if (lon < 0) {
      abs_lon = -lon;
      lon_ref = 'W';
    }

    /* All these tags are created with default values by exif_data_fix() */
    /* Change the data to the correct values for this image. */
    entry = init_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_PIXEL_X_DIMENSION);
    exif_set_long(entry->data, FILE_BYTE_ORDER, width);

    entry = init_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_PIXEL_Y_DIMENSION);
    exif_set_long(entry->data, FILE_BYTE_ORDER, height);

    entry = init_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_COLOR_SPACE);
    exif_set_short(entry->data, FILE_BYTE_ORDER, 1);

    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_LATITUDE_REF, 2);
    entry->format = EXIF_FORMAT_ASCII;
    entry->components = 2;
    //g_printf("data pointer %x, size=%d\n", entry->data, entry->size);
    entry->data[0] = lat_ref;
    entry->data[1] = '\0';

    //const char *lat_ref = 'N';
    //memcpy(entry->data, lat_ref, sizeof(lat_ref));

    /* Create tag for GPS Latitude */
    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_LATITUDE, 24);
    entry->format = EXIF_FORMAT_RATIONAL;
    entry->components= 3;
    ExifRational rational_lat = {(unsigned) roundf(abs_lat * GPS_PRECISION), GPS_PRECISION};
    exif_set_rational(entry->data, FILE_BYTE_ORDER, rational_lat);

    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_LONGITUDE_REF, 2);
    entry->format = EXIF_FORMAT_ASCII;
    entry->components = 2;
    //g_printf("data pointer %x, size=%d\n", entry->data, entry->size);
    entry->data[0] = lon_ref;
    entry->data[1] = '\0';

    /* Create tag for GPS Longitude */
    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_LONGITUDE, 24);
    entry->format = EXIF_FORMAT_RATIONAL;
    entry->components= 3;
    ExifRational rational_lon = {(unsigned) roundf(abs_lon * GPS_PRECISION), GPS_PRECISION};
    exif_set_rational(entry->data, FILE_BYTE_ORDER, rational_lon);

    double abs_alt = elv;
    ExifByte alt_ref = 0;

    if (abs_alt < 0) {
      alt_ref = 1;
      abs_alt = -abs_alt;
    }

    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE_REF, 1);
    entry->format = EXIF_FORMAT_BYTE;
    entry->components = 1;
    entry->data[0] = alt_ref;

    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE, 8);
    entry->format = EXIF_FORMAT_RATIONAL;
    entry->components= 1;
    ExifRational rational_alt = {(unsigned) roundf(elv), 1};
    exif_set_rational(entry->data, FILE_BYTE_ORDER, rational_alt);


    double speed_mph = speed * 0.621371;
    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_SPEED_REF, 2);
    entry->format = EXIF_FORMAT_ASCII;
    entry->components = 2;
    //g_printf("data pointer %x, size=%d\n", entry->data, entry->size);
    entry->data[0] = 'M';
    entry->data[1] = '\0';

    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_SPEED, 8);
    entry->format = EXIF_FORMAT_RATIONAL;
    entry->components= 1;
    ExifRational rational_speed = {(unsigned) roundf(speed_mph * 100000000), 100000000};
    exif_set_rational(entry->data, FILE_BYTE_ORDER, rational_speed);


    /* Create GPS Timestamp tag (hour, minute, second UTC) */
    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_TIME_STAMP, 24);
    entry->format = EXIF_FORMAT_RATIONAL;
    entry->components= 3;
    ExifRational rational_hour = {utc_hour, 1};
    exif_set_rational(entry->data, FILE_BYTE_ORDER, rational_hour);
    ExifRational rational_minute = {utc_minute, 1};
    exif_set_rational(&entry->data[8], FILE_BYTE_ORDER, rational_minute);
    ExifRational rational_second = {utc_second, 1};
    exif_set_rational(&entry->data[16], FILE_BYTE_ORDER, rational_second);

    /* Create GPS Data tag - ASCII representation of UTC year, month, day. */
    char time_buf[80];
    snprintf(time_buf, 80, "%04d:%02d:%02d", utc_year,
        utc_month, utc_day);
    size_t time_len = strlen(time_buf) + 1;
    entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_DATE_STAMP, time_len);
    entry->format = EXIF_FORMAT_ASCII;
    entry->components = time_len;
    strcpy((char *)entry->data, time_buf);

    /* Create file creation time from local time data (NTP server etc) */
    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(time_buf, 80, "%Y/%m/%d %r", timeinfo);
    LOG("%s\n", time_buf);

    time_len = strlen(time_buf) + 1;
    entry = create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME, time_len);
    entry->format = EXIF_FORMAT_ASCII;
    entry->components = time_len;
    strcpy((char *)entry->data, time_buf);


    /* Create a EXIF_TAG_USER_COMMENT tag. This one must be handled
   * differently because that tag isn't automatically created and
   * allocated by exif_data_fix(), nor can it be created using
   * exif_entry_initialize() so it must be explicitly allocated here.
   */
    entry = create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_USER_COMMENT, 
        sizeof(ASCII_COMMENT) + sizeof(FILE_COMMENT) - 2);
    /* Write the special header needed for a comment tag */
    memcpy(entry->data, ASCII_COMMENT, sizeof(ASCII_COMMENT)-1);
    /* Write the actual comment text, without the trailing NUL character */
    memcpy(entry->data+8, FILE_COMMENT, sizeof(FILE_COMMENT)-1);
    /* create_tag() happens to set the format and components correctly for
     * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */

    /* Get a pointer to the EXIF data block we just created */
    exif_data_save_data(exif, &exif_data, &exif_data_len);

    FILE *f = fopen(FILE_NAME, "wb");

    /* Write EXIF header */
    if (fwrite(exif_header, exif_header_len, 1, f) != 1) {
      ERR("Error writing to file %s\n", FILE_NAME);
    }
    /* Write EXIF block length in big-endian order */
    if (fputc((exif_data_len+2) >> 8, f) < 0) {
      ERR("Error writing to file %s\n", FILE_NAME);
    }
    if (fputc((exif_data_len+2) & 0xff, f) < 0) {
      ERR("Error writing to file %s\n", FILE_NAME);
    }
    /* Write EXIF data block */
    if (fwrite(exif_data, exif_data_len, 1, f) != 1) {
      ERR("Error writing to file %s\n", FILE_NAME);
    }
    /* Write JPEG image data, skipping the non-EXIF header */
    if (fwrite(((char *)image_data)+20, data_size-20, 1, f) != 1) {
      ERR("Error writing to file %s\n", FILE_NAME);
    }
    LOG("Wrote file %s\n", FILE_NAME);

    fclose(f);
    free(exif_data);
    exif_data_unref(exif);

    vdo_stream_buffer_unref(stream, &buffer, &error);

    LOG("Got frame: %p %d %d\n", image_data, width, height);
    LOG("Got GPS coordinates: %lf %c %lf %c, elevation=%lf, speed=%lf mph\n",
        abs_lat, lat_ref, abs_lon, lon_ref, elv, speed_mph);

    /* Send metadata stream info */
    metadata_send(abs_lat, lat_ref, abs_lon, lon_ref, speed_mph, "mph",
        elv, alt_ref);

    /* Upload file to FTP server */
    upload_ftp_image(FILE_NAME);

    return TRUE; /* FALSE removes the event source */
}

static void *log_position(void *data) 
{
  gint64 start_time, end_time;
  char *stmt;
  
  while(!exit_thread) {  
    start_time = g_get_monotonic_time();
    g_mutex_lock(&curr_position->m);
    if(curr_position->inited) {
      stmt = new_string("INSERT INTO gps_history VALUES (%lld, %f, %f)", g_get_real_time()/1000, curr_position->lat, curr_position->lon);
      sql_stmt(stmt);
      free_string(stmt);
    }
    //g_mutex_unlock(&curr_position->m);
    safe_unlock_mutex(&curr_position->m, "log_position 1");
    g_mutex_lock(&gps_log->m);
    if(gps_log->lines < gps_log->max_lines) {
      gps_log->lines = gps_log->lines + 1;
    }
    else {
      sql_stmt("DELETE FROM gps_history WHERE time = (SELECT MIN(time) FROM gps_history)");
    }
    end_time = start_time + gps_log->interval * G_TIME_SPAN_SECOND;
    //g_mutex_unlock(&gps_log->m);
    safe_unlock_mutex(&gps_log->m, "log_position 2");
    g_mutex_lock(&new_interval_m);
    while(g_cond_wait_until(&new_interval_c, &new_interval_m, end_time)) {
      if(exit_thread)
        return NULL;
      g_mutex_lock(&gps_log->m);
      end_time = start_time + gps_log->interval * G_TIME_SPAN_SECOND;
      safe_unlock_mutex(&gps_log->m, "log_position 3");
    }
    safe_unlock_mutex(&new_interval_m, "log_position 4");
  }
  return NULL;
}

static VdoStream* open_stream()
{
  GError* error      = NULL;

  VdoMap* settings = vdo_map_new();

  vdo_map_set_uint32(settings, "format", VDO_FORMAT_JPEG);

  stream = vdo_stream_new(settings, NULL, &error);

  g_clear_object(&settings);

  vdo_stream_attach(stream, NULL, &error);
    
  VdoMap* info = vdo_stream_get_info(stream, &error);

  syslog(LOG_INFO,
         "Starting stream: JPEG, %ux%u, %u fps\n",
         vdo_map_get_uint32(info, "width", 0),
         vdo_map_get_uint32(info, "height", 0),
         vdo_map_get_uint32(info, "framerate", 0));

  width  = vdo_map_get_uint32(info, "width", 0);
  height = vdo_map_get_uint32(info, "height", 0);

  vdo_stream_start(stream, &error);

  g_clear_object(&info);
  g_clear_error(&error);

  return stream;
}

int 
main(int argc, char *argv[])
{
  openlog(APP_ID, LOG_PID | LOG_CONS, LOG_USER);
  camera_init(APP_ID, APP_NICE_NAME);
  init_signals();

  FtpInit();

  loop = g_main_loop_new(NULL, FALSE);

  stream = open_stream();

  gps_log = g_new(GPSLog, 1);
  g_mutex_init(&gps_log->m);
  
  curr_position = g_new(GPSPosition, 1);
  g_mutex_init(&curr_position->m);
  curr_position->inited = FALSE;
  
  sql = g_new(SQLDB, 1);
  g_mutex_init(&sql->m);
  //sqlite3_open("./localdata/log.db", &sql->db);
  sqlite3_open("/tmp/gpslog.db", &sql->db);
  
  if(sql->db == 0) {
    ERR("Could not open database.");
    return EXIT_FAILURE;
  }
  
  sql_stmt("CREATE TABLE IF NOT EXISTS gps_history (time, lat, lon)");
  sqlite3_exec(sql->db, "SELECT COUNT(*) FROM gps_history", get_line_count, NULL, NULL);

  ftp_user     = g_strdup_printf(DEFAULT_FTP_USER);
  ftp_pass     = g_strdup_printf(DEFAULT_FTP_PASS);
  ftp_server   = g_strdup_printf(DEFAULT_FTP_SERVER);
  ftp_folder   = g_strdup_printf(DEFAULT_FTP_FOLDER);
  ftp_basename = g_strdup_printf(DEFAULT_FTP_BASENAME);
 
  char  value[50];
  if(camera_param_get("LogInterval", value, 50)) {
    gps_log->interval = atoi(value);
  }
  if(camera_param_get("LogCount", value, 50)) {
    set_log_count(value);
  }
  if(camera_param_get("ApiKey", value, 50)) {
    set_api_key(value);
  }
  if(camera_param_get("UDPPort", value, 50)) {
    set_udp_port(value);
  }
  if(camera_param_get("EXIFInt", value, 50)) {
    set_exif_int(value);
  }
  if(camera_param_get("FTPUser", value, 50)) {
    set_ftp_user(value);
  }
  if(camera_param_get("FTPPass", value, 50)) {
    set_ftp_pass(value);
  }
  if(camera_param_get("FTPServer", value, 50)) {
    set_ftp_server(value);
  }
  if(camera_param_get("FTPFolder", value, 50)) {
    set_ftp_folder(value);
  }
  if(camera_param_get("FTPBaseName", value, 50)) {
    set_ftp_basename(value);
  }
    

  camera_param_setCallback("LogInterval", set_log_interval);
  camera_param_setCallback("LogCount", set_log_count);
  camera_param_setCallback("ApiKey", set_api_key);
  camera_param_setCallback("UDPPort", set_udp_port);
  camera_param_setCallback("EXIFInt", set_exif_int);
  camera_param_setCallback("FTPUser", set_ftp_user);
  camera_param_setCallback("FTPPass", set_ftp_pass);
  camera_param_setCallback("FTPServer", set_ftp_server);
  camera_param_setCallback("FTPFolder", set_ftp_folder);
  camera_param_setCallback("FTPBaseName", set_ftp_basename);
  
  camera_http_setCallback("settings/get", api_settings_get);
  camera_http_setCallback("settings/set", api_settings_set);
  camera_http_setCallback("gps/now", api_position_now);
  //camera_http_setCallback("gps/random", api_position_random); // Only for testing
  camera_http_setCallback("gps/history", api_position_history);
  
  g_cond_init(&new_interval_c);
  g_mutex_init(&new_interval_m);

  exit_thread = FALSE;

  udp_init();

  GThread *log_thread = g_thread_new("gps_logger", &log_position, NULL);

  /* Periodically upload exif images */
  event_source_exif = g_timeout_add_seconds(exif_interval, exif_timeout, NULL);

  /* Initialize metadata streaming */
  metadata_stream_init();

  g_main_loop_run(loop);
  exit_thread = TRUE;
  g_cond_signal(&new_interval_c);

  g_thread_join(log_thread);
  g_main_loop_unref(loop);  
  
  sqlite3_close(sql->db);
  camera_cleanup();
  closelog();
  g_clear_object(&stream);

  return EXIT_SUCCESS;
}

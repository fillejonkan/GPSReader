PROG     = gpsrecv

CFLAGS   += -s -Ilibexif -Inmealib/include

PKGS = gio-2.0 glib-2.0 cairo axparameter axevent
CFLAGS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --cflags $(PKGS)) -DGETTEXT_PACKAGE=\"libexif-12\" -DLOCALEDIR=\"\"
LDLIBS += $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --libs $(PKGS))
LDFLAGS  += -s -laxoverlay -laxevent -laxparameter -laxhttp -lvdo -pthread

SRCS      = main.c camera/camera.c sqlite3/sqlite3.c ftplib.c metadata_stream.c overlay.c debug.c metadata_pair.c
SRCS     += $(wildcard libexif/*.c)
SRCS     += $(wildcard nmealib/src/*.c)
OBJS      = $(SRCS:.c=.o)

all: $(PROG) $(OBJS)

$(PROG): $(OBJS)
	$(CC) $^ $(CFLAGS) $(LIBS) $(LDFLAGS) -lm -ldl $(LDLIBS) -o $@
	$(STRIP) $@

clean:
	rm -f $(PROG) $(OBJS)



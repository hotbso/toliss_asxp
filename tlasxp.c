/*
MIT License

Copyright (c) 2019, 2021, 2022, 2023 Holger Teutsch

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <time.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>

#include "XPLMPlugin.h"
#include "XPLMPlanes.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include "tlasxp.h"

#define UNUSED(x) (void)(x)

#define VERSION "1.0a-dev"

#define LB_2_KG 0.45359237    /* imperial to metric */

static float flight_loop_cb(float unused1, float unused2, int unused3, void *unused4);

static char xpdir[512];
static const char *psep;
static char fms_path[512];

char tlasxp_tmp_fn[512];

static XPLMMenuID tlasxp_menu;

#define MSG_GET_OFP (xpMsg_UserStart + 1)
static XPWidgetID getofp_widget, display_widget, getofp_btn,
                  status_line,
                  xfer_fuel_btn, xfer_payload_btn, xfer_all_btn;
static XPWidgetID conf_widget, conf_downl_fpl_btn, pilot_id_input, conf_ok_btn;

#ifdef UPLOAD_ASXP
static XPWidgetID conf_upl_aspx_btn;
#endif

typedef struct _widget_ctx
{
    XPWidgetID widget;
    int in_vr;          /* currently in vr */
    int l, t, w, h;     /* last geometry before bringing into vr */
} widget_ctx_t;

static widget_ctx_t getofp_widget_ctx, conf_widget_ctx;

static ofp_info_t ofp_info;

static XPLMDataRef vr_enabled_dr,
                   acf_icao_dr,
                   mcdu1_spw_dr, mcdu2_spw_dr;

static XPLMCreateFlightLoop_t create_flight_loop =
{
    .structSize = sizeof(XPLMCreateFlightLoop_t),
    .phase = xplm_FlightLoop_Phase_BeforeFlightModel,
    .callbackFunc = flight_loop_cb
};
static XPLMFlightLoopID flight_loop_id;

static int dr_mapped;
static int error_disabled;

static char pref_path[512];
static char pilot_id[20];
static int flag_download_fms, flag_upload_aspx;
static char acf_file[256];
static char acf_icao[41];
static char msg_line_1[100], msg_line_2[100], msg_line_3[100];


static void
map_datarefs()
{
    if (dr_mapped)
        return;

    if (NULL == (mcdu1_spw_dr = XPLMFindDataRef("AirbusFBW/MCDU1spw"))) goto err;
    if (NULL == (mcdu2_spw_dr = XPLMFindDataRef("AirbusFBW/MCDU2spw"))) goto err;

    dr_mapped = 1;
    return;

err:
    log_msg("Can't map all datarefs, disabled");
}

static void
save_pref()
{
    FILE *f = fopen(pref_path, "wb");
    if (NULL == f)
        return;

    fputs(pilot_id, f); putc('\n', f);
    putc((flag_download_fms ? '1' : '0'), f); putc('\n', f);
    putc((flag_upload_aspx ? '1' : '0'), f); putc('\n', f);
    fclose(f);
}


static void
load_pref()
{
    char c;
    FILE *f  = fopen(pref_path, "rb");
    if (NULL == f)
        return;

    fgets(pilot_id, sizeof(pilot_id), f);
    int len = strlen(pilot_id);
    if ('\n' == pilot_id[len - 1]) pilot_id[len - 1] = '\0';

    if (EOF == (c = fgetc(f))) goto out;
    flag_download_fms = (c == '1' ? 1 : 0);
    fgetc(f); /* skip over \n */

    if (EOF == (c = fgetc(f))) goto out;
    flag_upload_aspx = (c == '1' ? 1 : 0);

  out:
    flag_upload_aspx &= flag_download_fms;
    fclose(f);
}

static void
download_fms()
{
    char URL[300], fn[500];
    FILE *f = NULL;

    tlasxp_dump_ofp_info(&ofp_info);

    snprintf(URL, sizeof(URL), "%s%s", ofp_info.sb_path, ofp_info.sb_fms_link);
    log_msg("URL '%s'", URL);
    snprintf(fn, sizeof(fn), "%s%s%s%s19.fms", fms_path, psep, ofp_info.origin, ofp_info.destination);

    if (NULL == (f = fopen(fn, "wb"))) {
        log_msg("Can't create file '%s'", fn);
        goto err_out;
    }

    if (0 == tlasxp_http_get(URL, f, NULL, 10)) {
        log_msg("Can't download '%s'", URL);
        goto err_out;
    }

    snprintf(msg_line_2, sizeof(msg_line_2), "FMS plan: '%s%s19'", ofp_info.origin, ofp_info.destination);

    snprintf(URL, sizeof(URL), "http://localhost:19285/ActiveSky/API/LoadFlightPlan?FileName=%s%s19.fms",
                               ofp_info.origin, ofp_info.destination);
    log_msg("URL '%s'", URL);

    if (0 == tlasxp_http_get(URL, NULL, NULL, 2)) {
        log_msg("Can't upload to ASXP '%s'", URL);
        strcpy(msg_line_3, "Could not upload flightplan to ASXP");
    } else {
        strcpy(msg_line_3, "Flightplan uploaded to ASXP");
    }

  err_out:
    if (f) fclose(f);
}


static void
show_widget(widget_ctx_t *ctx)
{
    if (XPIsWidgetVisible(ctx->widget))
        return;

    /* force window into visible area of screen
       we use modern windows under the hut so UI coordinates are in boxels */

    int xl, yl, xr, yr;
    XPLMGetScreenBoundsGlobal(&xl, &yr, &xr, &yl);

    ctx->l = (ctx->l + ctx->w < xr) ? ctx->l : xr - ctx->w - 50;
    ctx->l = (ctx->l <= xl) ? 20 : ctx->l;

    ctx->t = (ctx->t + ctx->h < yr) ? ctx->t : (yr - ctx->h - 50);
    ctx->t = (ctx->t >= ctx->h) ? ctx->t : (yr / 2);

    log_msg("show_widget: s: (%d, %d) -> (%d, %d), w: (%d, %d) -> (%d,%d)",
           xl, yl, xr, yr, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);

    XPSetWidgetGeometry(ctx->widget, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);
    XPShowWidget(ctx->widget);

    int in_vr = (NULL != vr_enabled_dr) && XPLMGetDatai(vr_enabled_dr);
    if (in_vr) {
        log_msg("VR mode detected");
        XPLMWindowID window =  XPGetWidgetUnderlyingWindow(ctx->widget);
        XPLMSetWindowPositioningMode(window, xplm_WindowVR, -1);
        ctx->in_vr = 1;
    } else {
        if (ctx->in_vr) {
            log_msg("widget now out of VR, map at (%d,%d)", ctx->l, ctx->t);
            XPLMWindowID window =  XPGetWidgetUnderlyingWindow(ctx->widget);
            XPLMSetWindowPositioningMode(window, xplm_WindowPositionFree, -1);

            /* A resize is necessary so it shows up on the main screen again */
            XPSetWidgetGeometry(ctx->widget, ctx->l, ctx->t, ctx->l + ctx->w, ctx->t - ctx->h);
            ctx->in_vr = 0;
        }
    }
}


static int
conf_widget_cb(XPWidgetMessage msg, XPWidgetID widget_id, intptr_t param1, intptr_t param2)
{
    if (msg == xpMessage_CloseButtonPushed) {
        XPHideWidget(widget_id);
        return 1;
    }

    if (error_disabled)
        return 1;

    if ((widget_id == conf_ok_btn) && (msg == xpMsg_PushButtonPressed)) {
        XPGetWidgetDescriptor(pilot_id_input, pilot_id, sizeof(pilot_id));
        flag_download_fms = XPGetWidgetProperty(conf_downl_fpl_btn, xpProperty_ButtonState, NULL);
        flag_upload_aspx = XPGetWidgetProperty(conf_upl_aspx_btn, xpProperty_ButtonState, NULL);
        flag_upload_aspx &= flag_download_fms;
        save_pref();
        XPHideWidget(conf_widget);
        return 1;
    }


    return 0;
}

/* return success == 1 */
static int
fetch_ofp(void)
{
    msg_line_1[0] = msg_line_2[0] = msg_line_3[0] = '\0';

    ofp_info.valid = 0;

    tlasxp_ofp_get_parse(pilot_id, &ofp_info);
    tlasxp_dump_ofp_info(&ofp_info);

    if (strcmp(ofp_info.status, "Success")) {
        //XPSetWidgetDescriptor(status_line, ofp_info.status);
        return 0; // error
    }


    ofp_info.valid = 1;

    download_fms();
    return 1;
}

static int
getofp_widget_cb(XPWidgetMessage msg, XPWidgetID widget_id, intptr_t param1, intptr_t param2)
{
    if (msg == xpMessage_CloseButtonPushed) {
        XPHideWidget(widget_id);
        return 1;
    }

    if (error_disabled)
        return 1;

    if ((widget_id == getofp_btn) && (msg == xpMsg_PushButtonPressed)) {
        XPSetWidgetDescriptor(status_line, "Fetching...");
        /* Send message to myself to get a draw cycle (draws button as selected) */
        XPSendMessageToWidget(status_line, MSG_GET_OFP, xpMode_UpChain, 0, 0);
        return 1;
    }

    /* self sent message: fetch OFP (lengthy) */
    if ((widget_id == getofp_widget) && (MSG_GET_OFP == msg)) {
        (void)fetch_ofp();
        return 1;
    }

    return 0;
}

static void
create_widget()
{
    if (getofp_widget)
        return;

    int left = 200;
    int top = 800;
    int width = 450;
    int height = 300;

    getofp_widget_ctx.l = left;
    getofp_widget_ctx.t = top;
    getofp_widget_ctx.w = width;
    getofp_widget_ctx.h = height;

    getofp_widget = XPCreateWidget(left, top, left + width, top - height,
                                 0, "Toliss ASXP Connector " VERSION, 1, NULL, xpWidgetClass_MainWindow);
    getofp_widget_ctx.widget = getofp_widget;

    XPSetWidgetProperty(getofp_widget, xpProperty_MainWindowHasCloseBoxes, 1);
    XPAddWidgetCallback(getofp_widget, getofp_widget_cb);
    left += 5; top -= 25;

    int left1 = left + 10;
    getofp_btn = XPCreateWidget(left1, top, left1 + 60, top - 30,
                              1, "Fetch OFP", 0, getofp_widget, xpWidgetClass_Button);
    XPAddWidgetCallback(getofp_btn, getofp_widget_cb);

    top -= 25;
    status_line = XPCreateWidget(left1, top, left + width - 10, top - 20,
                              1, "", 0, getofp_widget, xpWidgetClass_Caption);

    top -= 20;
    display_widget = XPCreateCustomWidget(left + 10, top, left + width -20, top - height + 10,
                                           1, "", 0, getofp_widget, getofp_widget_cb);
    top -= 50;
    left1 = left + 10;
    xfer_fuel_btn = XPCreateWidget(left1, top, left1 + 50, top - 30,
                              1, "Refuel", 0, getofp_widget, xpWidgetClass_Button);
    XPAddWidgetCallback(xfer_fuel_btn, getofp_widget_cb);

    left1 += 60;
    xfer_payload_btn = XPCreateWidget(left1, top, left1 + 50, top - 30,
                              1, "Board", 0, getofp_widget, xpWidgetClass_Button);
    XPAddWidgetCallback(xfer_payload_btn, getofp_widget_cb);

    left1 += 60;
    xfer_all_btn = XPCreateWidget(left1, top, left1 + 150, top - 30,
                              1, "Xfer Load data to ISCS", 0, getofp_widget, xpWidgetClass_Button);
    XPAddWidgetCallback(xfer_all_btn, getofp_widget_cb);
}

static void
menu_cb(void *menu_ref, void *item_ref)
{
    /* create gui */
    if (item_ref == &getofp_widget) {
        create_widget();
        show_widget(&getofp_widget_ctx);
        return;
    }

    if (item_ref == &conf_widget) {
        if (NULL == conf_widget) {
            int left = 250;
            int top = 780;
            int width = 500;
#ifdef UPLOAD_ASXP
            int height = 220;
#else
             int height = 180;
#endif

            conf_widget_ctx.l = left;
            conf_widget_ctx.t = top;
            conf_widget_ctx.w = width;
            conf_widget_ctx.h = height;

            conf_widget = XPCreateWidget(left, top, left + width, top - height,
                                         0, "Toliss ASXP Connector / Configuration", 1, NULL, xpWidgetClass_MainWindow);
            conf_widget_ctx.widget = conf_widget;

            XPSetWidgetProperty(conf_widget, xpProperty_MainWindowHasCloseBoxes, 1);
            XPAddWidgetCallback(conf_widget, conf_widget_cb);
            left += 5; top -= 25;
            XPCreateWidget(left, top, left + width - 2 * 5, top - 15,
                           1, "Pilot Id", 0, conf_widget, xpWidgetClass_Caption);

            int left1 = left + 60;
            pilot_id_input = XPCreateWidget(left1, top, left1 +  50, top - 15,
                                            1, pilot_id, 0, conf_widget, xpWidgetClass_TextField);
            XPSetWidgetProperty(pilot_id_input, xpProperty_TextFieldType, xpTextEntryField);
            XPSetWidgetProperty(pilot_id_input, xpProperty_MaxCharacters, sizeof(pilot_id) -1);

            top -= 20;
            XPCreateWidget(left, top, left + width - 10, top - 20,
                                      1, "Download Flightplan", 0, conf_widget, xpWidgetClass_Caption);
            top -= 20;
            conf_downl_fpl_btn = XPCreateWidget(left, top, left + 20, top - 20,
                                      1, "", 0, conf_widget, xpWidgetClass_Button);
            XPSetWidgetProperty(conf_downl_fpl_btn, xpProperty_ButtonType, xpRadioButton);
            XPSetWidgetProperty(conf_downl_fpl_btn, xpProperty_ButtonBehavior, xpButtonBehaviorCheckBox);


            top -= 30;
            conf_ok_btn = XPCreateWidget(left + 10, top, left + 140, top - 30,
                                      1, "OK", 0, conf_widget, xpWidgetClass_Button);
            XPAddWidgetCallback(conf_ok_btn, conf_widget_cb);
        }

        XPSetWidgetDescriptor(pilot_id_input, pilot_id);
        XPSetWidgetProperty(conf_downl_fpl_btn, xpProperty_ButtonState, flag_download_fms);

#ifdef UPLOAD_ASXP
        XPSetWidgetProperty(conf_upl_aspx_btn, xpProperty_ButtonState, flag_upload_aspx);
#endif

        show_widget(&conf_widget_ctx);
        return;
    }
}

/* call back for fetch cmd */
static int
fetch_cmd_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(ref);
    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("fetch cmd called");
    create_widget();
    fetch_ofp();
    show_widget(&getofp_widget_ctx);
    return 0;
}

/* call back for fetch_xfer cmd */
static int
fetch_xfer_cmd_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(ref);
    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("fetch_xfer cmd called");

    if (0 == fetch_ofp()) {
        /* error, show widget */
        create_widget();
        show_widget(&getofp_widget_ctx);
        return 0;
    }
    return 0;
}

/* call back for toggle cmd */
static int
toggle_cmd_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(ref);
    if (xplm_CommandBegin != phase)
        return 0;

    log_msg("toggle cmd called");
    create_widget();

    if (XPIsWidgetVisible(getofp_widget_ctx.widget)) {
        XPHideWidget(getofp_widget_ctx.widget);
        return 0;
    }

    show_widget(&getofp_widget_ctx);
    return 0;
}

static int aoc_init_done;

/* flight loop for delayed actions */
static float
flight_loop_cb(float unused1, float unused2, int unused3, void *unused4)
{
    if (aoc_init_done)
        return 0;

    map_datarefs();
    if (! dr_mapped)
        return 10.0;

    char buf[26];
    int l = XPLMGetDatab(mcdu1_spw_dr, buf, 0, sizeof(buf) - 1);
    buf[l] = '\0';
    //log_msg("MCDU1 '%s'", buf);
    if (0 == strcmp(buf, "AOC ACT F-PLN UPLINK")) {
        aoc_init_done = 1;
    } else {
        l = XPLMGetDatab(mcdu2_spw_dr, buf, 0, sizeof(buf) - 1);
        buf[l] = '\0';
        //log_msg("MCDU2 '%s'", buf);
        if (0 == strcmp(buf, "AOC ACT F-PLN UPLINK"))
            aoc_init_done = 1;
    }

    if (aoc_init_done) {
        log_msg("AOC init detected");
        fetch_ofp();
        return 0;
    }

    return 2.0;
}

//* ------------------------------------------------------ API -------------------------------------------- */
PLUGIN_API int
XPluginStart(char *out_name, char *out_sig, char *out_desc)
{
    log_msg("startup " VERSION);

    /* Always use Unix-native paths on the Mac! */
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);

    strcpy(out_name, "toliss_aspx " VERSION);
    strcpy(out_sig, "tlasxp-hotbso");
    strcpy(out_desc, "A plugin that interfaces ASPX to ToLiss A3xx");

    psep = XPLMGetDirectorySeparator();
    XPLMGetSystemPath(xpdir);
    snprintf(fms_path, sizeof(fms_path), "%s%sOutput%sFMS plans%s",
             xpdir, psep, psep, psep);

    snprintf(tlasxp_tmp_fn, sizeof(tlasxp_tmp_fn), "%s%sOutput%stlasxp_download.tmp",
             xpdir, psep, psep);

    /* map standard datarefs, acf datarefs are delayed */
    vr_enabled_dr = XPLMFindDataRef("sim/graphics/VR/enabled");
    acf_icao_dr = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");

    /* load preferences */
    XPLMGetPrefsPath(pref_path);
    XPLMExtractFileAndPath(pref_path);
    strcat(pref_path, psep);
    strcat(pref_path, "toliss_asxp.prf");
    load_pref();
    return 1;
}


PLUGIN_API void
XPluginStop(void)
{
}


PLUGIN_API void
XPluginDisable(void)
{
    if (flight_loop_id)
        XPLMScheduleFlightLoop(flight_loop_id, 0.0, 0);
}


PLUGIN_API int
XPluginEnable(void)
{
    if (flight_loop_id)
        XPLMScheduleFlightLoop(flight_loop_id, 0.0, 0);
    return 1;
}


PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID in_from, long in_msg, void *in_param)
{
    UNUSED(in_from);

    switch (in_msg) {
        case XPLM_MSG_PLANE_LOADED:
            if (in_param == 0) {
                char path[512];

                XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, path);
                log_msg(acf_file);

                acf_file[4] = '\0';
                for (int i = 0; i < 4; i++)
                    acf_file[i] = toupper(acf_file[i]);

                if ((0 == strcmp(acf_file, "A319")) || (0 == strcmp(acf_file, "A321")) ||
                    (0 == strcmp(acf_file, "A340"))) {
                    XPLMGetSystemPath(path);
                    char *s = path + strlen(path);

                    /* check for directory */
                    sprintf(s, "Resources%splugins%sToLissData%sSituations", psep, psep, psep);
                    if (0 != access(path, F_OK))
                        return;

                    log_msg("Detected ToLiss A319/A321/A340");
                    int l = XPLMGetDatab(acf_icao_dr, acf_icao, 0, sizeof(acf_icao) - 1);
                    acf_icao[l] = '\0';
                    log_msg("ToLiss ICAO is %d, %s", l, acf_icao);

                    if (NULL == tlasxp_menu) {
                        XPLMMenuID menu = XPLMFindPluginsMenu();
                        int sub_menu = XPLMAppendMenuItem(menu, "ASXP Connector", NULL, 1);
                        tlasxp_menu = XPLMCreateMenu("ASXP Connector", menu, sub_menu, menu_cb, NULL);
                        XPLMAppendMenuItem(tlasxp_menu, "Configure", &conf_widget, 0);
                        XPLMAppendMenuItem(tlasxp_menu, "Show widget", &getofp_widget, 0);

                        XPLMCommandRef cmdr = XPLMCreateCommand("tlasxp/toggle", "Toggle ASXP connector widget");
                        XPLMRegisterCommandHandler(cmdr, toggle_cmd_cb, 0, NULL);

                        cmdr = XPLMCreateCommand("tlasxp/fetch", "Fetch ofp data and show in widget");
                        XPLMRegisterCommandHandler(cmdr, fetch_cmd_cb, 0, NULL);

                        cmdr = XPLMCreateCommand("tlasxp/fetch_xfer", "Fetch ofp data and xfer load data");
                        XPLMRegisterCommandHandler(cmdr, fetch_xfer_cmd_cb, 0, NULL);

                        flight_loop_id = XPLMCreateFlightLoop(&create_flight_loop);
                        XPLMScheduleFlightLoop(flight_loop_id, 10.0, 1);
                    }
               } else {
                   if (flight_loop_id)
                        XPLMScheduleFlightLoop(flight_loop_id, 0.0, 0);
               }
            }
        break;
    }
}
